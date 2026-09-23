package com.minesafety.roboeye.nav.planner

import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import java.util.PriorityQueue
import kotlin.math.abs
import kotlin.math.hypot

/**
 * 2D spatial waypoint along a planned route.
 */
data class Waypoint(
    val xM: Float,
    val yM: Float,
    val gridX: Int = 0,
    val gridY: Int = 0,
)

/**
 * Validated collision-free path planned across the local spatial map.
 */
data class PlannedPath(
    val waypoints: List<Waypoint>,
    val totalLengthM: Float,
    val generatedAtMs: Long = System.currentTimeMillis(),
) {
    val isEmpty: Boolean get() = waypoints.isEmpty()
    val size: Int get() = waypoints.size

    companion object {
        val EMPTY = PlannedPath(emptyList(), 0.0f)
    }
}

/**
 * Outcome of a path planning query.
 */
sealed interface PlanningResult {
    data class Success(
        val path: PlannedPath,
        val expandedNodes: Int,
        val planningTimeMs: Long,
    ) : PlanningResult

    data class Failure(
        val reason: String,
        val expandedNodes: Int = 0,
        val planningTimeMs: Long = 0L,
    ) : PlanningResult
}

/**
 * Deterministic, bounded A* path planner operating over the 24x24 occupancy grid.
 *
 * Enforces:
 * 1. "UNKNOWN IS NOT SAFE" — uncertain / unknown cells are rejected as untraversable.
 * 2. Rover Footprint Inflation — obstacle cells are expanded by the rover bounding radius + safety margin.
 * 3. Bounded Iterations — capped at [MAX_ITERATIONS] (500) to guarantee zero compute spikes on Cortex-A53.
 * 4. Deterministic Tie-Breaking — strictly deterministic priority order.
 * 5. Lightweight Path Smoothing — line-of-sight shortcutting without heavy solvers.
 */
class AStarGridPlanner(
    private val maxIterations: Int = 500,
) {
    private data class Node(
        val gx: Int,
        val gy: Int,
        val gCost: Float,
        val fCost: Float,
        val parent: Node? = null,
    ) : Comparable<Node> {
        override fun compareTo(other: Node): Int {
            val c = this.fCost.compareTo(other.fCost)
            return if (c != 0) c else this.gCost.compareTo(other.gCost)
        }
    }

    /**
     * Plans a collision-free path from current pose to [goal] using the world model.
     */
    fun planPath(worldModel: NavigationWorldModel, goal: LocalGoal): PlanningResult {
        val startTime = System.currentTimeMillis()
        val map = worldModel.spatialMap

        val startGx = ((worldModel.pose.xM / map.resolutionM) + map.gridWidth / 2f).toInt()
            .coerceIn(0, map.gridWidth - 1)
        val startGy = ((worldModel.pose.yM / map.resolutionM) + map.gridHeight / 2f).toInt()
            .coerceIn(0, map.gridHeight - 1)

        val goalGx = ((goal.targetX / map.resolutionM) + map.gridWidth / 2f).toInt()
        val goalGy = ((goal.targetY / map.resolutionM) + map.gridHeight / 2f).toInt()

        if (goalGx !in 0 until map.gridWidth || goalGy !in 0 until map.gridHeight) {
            return PlanningResult.Failure(
                "Goal ($goalGx, $goalGy) outside map bounds",
                planningTimeMs = System.currentTimeMillis() - startTime,
            )
        }

        // Build footprint-inflated collision grid
        val inflatedBlocked = buildInflatedCollisionGrid(worldModel)

        // Validate goal cell isn't blocked
        if (inflatedBlocked[goalGx][goalGy]) {
            return PlanningResult.Failure(
                "Goal is inside an obstacle or safety-inflated footprint buffer",
                planningTimeMs = System.currentTimeMillis() - startTime,
            )
        }

        // Search state
        val openList = PriorityQueue<Node>()
        val closedList = BooleanArray(map.gridWidth * map.gridHeight)
        val gCosts = FloatArray(map.gridWidth * map.gridHeight) { Float.MAX_VALUE }

        fun idx(x: Int, y: Int): Int = y * map.gridWidth + x

        val h0 = heuristic(startGx, startGy, goalGx, goalGy)
        val startNode = Node(startGx, startGy, 0.0f, h0)
        openList.add(startNode)
        gCosts[idx(startGx, startGy)] = 0.0f

        var iterations = 0
        var goalNode: Node? = null

        // 8-connected neighbor offsets
        val dx = intArrayOf(0, 0, 1, -1, 1, 1, -1, -1)
        val dy = intArrayOf(1, -1, 0, 0, 1, -1, 1, -1)
        val moveCosts = floatArrayOf(1.0f, 1.0f, 1.0f, 1.0f, 1.4142f, 1.4142f, 1.4142f, 1.4142f)

        while (openList.isNotEmpty() && iterations < maxIterations) {
            iterations++
            val current = openList.poll() ?: break

            if (current.gx == goalGx && current.gy == goalGy) {
                goalNode = current
                break
            }

            val curIdx = idx(current.gx, current.gy)
            if (closedList[curIdx]) continue
            closedList[curIdx] = true

            for (i in 0 until 8) {
                val nx = current.gx + dx[i]
                val ny = current.gy + dy[i]

                if (nx !in 0 until map.gridWidth || ny !in 0 until map.gridHeight) continue
                val nIdx = idx(nx, ny)
                if (closedList[nIdx]) continue

                // Check collision (inflated obstacle or unknown)
                // Exception: allow start node position itself
                if (inflatedBlocked[nx][ny] && !(nx == startGx && ny == startGy)) continue

                // Corner cutting prevention for diagonal moves
                if (i >= 4) {
                    if (inflatedBlocked[current.gx + dx[i]][current.gy] ||
                        inflatedBlocked[current.gx][current.gy + dy[i]]
                    ) {
                        continue
                    }
                }

                val tentativeG = current.gCost + moveCosts[i]
                if (tentativeG < gCosts[nIdx]) {
                    gCosts[nIdx] = tentativeG
                    val fCost = tentativeG + heuristic(nx, ny, goalGx, goalGy)
                    openList.add(Node(nx, ny, tentativeG, fCost, current))
                }
            }
        }

        val elapsed = System.currentTimeMillis() - startTime
        if (goalNode == null) {
            return PlanningResult.Failure(
                if (iterations >= maxIterations) "A* exceeded search budget ($maxIterations steps)" else "No collision-free path exists to goal",
                expandedNodes = iterations,
                planningTimeMs = elapsed,
            )
        }

        // Reconstruct raw grid waypoints
        val rawWaypoints = mutableListOf<Waypoint>()
        var curr: Node? = goalNode
        while (curr != null) {
            val xm = (curr.gx - map.gridWidth / 2f) * map.resolutionM
            val ym = (curr.gy - map.gridHeight / 2f) * map.resolutionM
            rawWaypoints.add(Waypoint(xm, ym, curr.gx, curr.gy))
            curr = curr.parent
        }
        rawWaypoints.reverse()

        // Apply lightweight path smoothing
        val smoothed = smoothPath(rawWaypoints, inflatedBlocked, map)
        val totalLength = calculatePathLength(smoothed)

        return PlanningResult.Success(
            path = PlannedPath(smoothed, totalLength, System.currentTimeMillis()),
            expandedNodes = iterations,
            planningTimeMs = elapsed,
        )
    }

    /**
     * Builds footprint-inflated collision grid where blocked = true if cell is OCCUPIED
     * or within [worldModel.inflationRadiusM] of an obstacle, or is UNCERTAIN ("UNKNOWN IS NOT SAFE").
     */
    fun buildInflatedCollisionGrid(worldModel: NavigationWorldModel): Array<BooleanArray> {
        val map = worldModel.spatialMap
        val blocked = Array(map.gridWidth) { BooleanArray(map.gridHeight) }
        val inflationCells = (worldModel.inflationRadiusM / map.resolutionM).toInt().coerceAtLeast(1)

        // 1. Mark obstacles & unknown regions as untraversable
        for (gx in 0 until map.gridWidth) {
            for (gy in 0 until map.gridHeight) {
                val cell = map.cells[gx][gy]
                if (cell.state == MapCellState.OCCUPIED) {
                    // Inflate obstacle by footprint radius
                    for (ix in -inflationCells..inflationCells) {
                        for (iy in -inflationCells..inflationCells) {
                            if (hypot(ix.toDouble(), iy.toDouble()) <= inflationCells) {
                                val nx = gx + ix
                                val ny = gy + iy
                                if (nx in 0 until map.gridWidth && ny in 0 until map.gridHeight) {
                                    blocked[nx][ny] = true
                                }
                            }
                        }
                    }
                } else if (cell.state == MapCellState.UNCERTAIN) {
                    // UNKNOWN IS NOT SAFE: unknown cells are blocked
                    blocked[gx][gy] = true
                }
            }
        }
        return blocked
    }

    /**
     * Line-of-sight path smoothing shortcutting redundant intermediate grid nodes.
     */
    private fun smoothPath(
        waypoints: List<Waypoint>,
        inflatedBlocked: Array<BooleanArray>,
        map: LocalSpatialMap,
    ): List<Waypoint> {
        if (waypoints.size <= 2) return waypoints

        val smoothed = mutableListOf<Waypoint>()
        smoothed.add(waypoints.first())

        var currentIdx = 0
        while (currentIdx < waypoints.size - 1) {
            var furthestVisible = currentIdx + 1
            for (candidate in (waypoints.size - 1) downTo (currentIdx + 2)) {
                if (hasLineOfSight(waypoints[currentIdx], waypoints[candidate], inflatedBlocked, map)) {
                    furthestVisible = candidate
                    break
                }
            }
            smoothed.add(waypoints[furthestVisible])
            currentIdx = furthestVisible
        }

        return smoothed
    }

    /**
     * Bresenham / Raycast line-of-sight test between two waypoints.
     */
    private fun hasLineOfSight(
        w1: Waypoint,
        w2: Waypoint,
        inflatedBlocked: Array<BooleanArray>,
        map: LocalSpatialMap,
    ): Boolean {
        val dist = hypot(w2.xM - w1.xM, w2.yM - w1.yM)
        val steps = (dist / (map.resolutionM * 0.5f)).toInt().coerceAtLeast(2)

        for (s in 1 until steps) {
            val t = s.toFloat() / steps
            val xm = w1.xM + (w2.xM - w1.xM) * t
            val ym = w1.yM + (w2.yM - w1.yM) * t

            val gx = ((xm / map.resolutionM) + map.gridWidth / 2f).toInt()
            val gy = ((ym / map.resolutionM) + map.gridHeight / 2f).toInt()

            if (gx !in 0 until map.gridWidth || gy !in 0 until map.gridHeight) return false
            if (inflatedBlocked[gx][gy]) return false
        }
        return true
    }

    private fun heuristic(x1: Int, y1: Int, x2: Int, y2: Int): Float {
        // Euclidean distance with tie-breaker factor
        return hypot((x1 - x2).toDouble(), (y1 - y2).toDouble()).toFloat() * 1.001f
    }

    private fun calculatePathLength(waypoints: List<Waypoint>): Float {
        var total = 0.0f
        for (i in 0 until waypoints.size - 1) {
            total += hypot(
                waypoints[i + 1].xM - waypoints[i].xM,
                waypoints[i + 1].yM - waypoints[i].yM,
            )
        }
        return total
    }
}
