package com.minesafety.roboeye.nav

import com.minesafety.roboeye.localization.LocalPose
import com.minesafety.roboeye.localization.TrackingQuality
import com.minesafety.roboeye.mapping.LocalSpatialMap
import com.minesafety.roboeye.mapping.MapCellState
import com.minesafety.roboeye.nav.model.GoalValidator
import com.minesafety.roboeye.nav.model.LocalGoal
import com.minesafety.roboeye.nav.model.NavigationWorldModel
import com.minesafety.roboeye.nav.planner.AStarGridPlanner
import com.minesafety.roboeye.nav.planner.PathValidator
import com.minesafety.roboeye.nav.planner.PlanningResult
import com.minesafety.roboeye.vision.GeometryTrustLevel
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertTrue
import org.junit.Before
import org.junit.Test

/**
 * Deterministic unit tests covering:
 * A. Goal validation
 * B. A* path generation
 * C. Blocked path (detour around obstacle)
 * D. Unknown-cell rejection ("UNKNOWN IS NOT SAFE")
 * E. Rover footprint collision (narrow gap rejection)
 * F. Path validity check
 * G. Path replanning on obstacle emergence
 * H. Goal reached tolerance
 */
class GoalAndPlanningTest {

    private lateinit var map: LocalSpatialMap
    private lateinit var planner: AStarGridPlanner
    private lateinit var pathValidator: PathValidator

    @Before
    fun setUp() {
        map = LocalSpatialMap(gridWidth = 24, gridHeight = 24, resolutionM = 0.25f)
        planner = AStarGridPlanner(maxIterations = 500)
        pathValidator = PathValidator(maxPathAgeMs = 5000L)
    }

    private fun createWorldModel(
        pose: LocalPose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.TRACKING),
        trust: GeometryTrustLevel = GeometryTrustLevel.TRUSTED,
    ): NavigationWorldModel {
        return NavigationWorldModel(
            pose = pose,
            spatialMap = map,
            geometryTrust = trust,
            timestampMs = System.currentTimeMillis(),
        )
    }

    private fun markFreeCorridor(startX: Int, endX: Int, startY: Int, endY: Int) {
        for (x in startX..endX) {
            for (y in startY..endY) {
                map.cells[x][y].state = MapCellState.FREE
                map.cells[x][y].confidence = 0.90f
            }
        }
    }

    // --- A. GOAL VALIDATION TESTS ---
    @Test
    fun testGoalValidation_AcceptsValidGoalInFreeSpace() {
        // Mark area around (1.0m, 0.0m) as free (grid center is at (12, 12))
        markFreeCorridor(10, 18, 10, 14)
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)

        val result = GoalValidator.validate(goal, wm)
        assertTrue("Expected valid goal in free space", result.isValid)
    }

    @Test
    fun testGoalValidation_RejectsGoalOutOfBounds() {
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 5.0f, targetY = 0.0f) // Outside 6m x 6m bounds

        val result = GoalValidator.validate(goal, wm)
        assertFalse(result.isValid)
        assertTrue(result.reason?.contains("outside") == true)
    }

    @Test
    fun testGoalValidation_RejectsGoalInOccupiedCell() {
        markFreeCorridor(10, 18, 10, 14)
        // Mark cell at (1.0m, 0.0m) -> (16, 12) as OCCUPIED
        map.cells[16][12].state = MapCellState.OCCUPIED
        map.cells[16][12].confidence = 0.95f

        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)

        val result = GoalValidator.validate(goal, wm)
        assertFalse(result.isValid)
        assertTrue(result.reason?.contains("occupied") == true)
    }

    @Test
    fun testGoalValidation_RejectsGoalInUncertainCell_UnknownIsNotSafe() {
        // Leave map as default UNCERTAIN
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)

        val result = GoalValidator.validate(goal, wm)
        assertFalse("Goal in uncertain cell must be rejected: UNKNOWN IS NOT SAFE", result.isValid)
        assertTrue(result.reason?.contains("unknown", ignoreCase = true) == true ||
                   result.reason?.contains("uncertain", ignoreCase = true) == true)
    }

    @Test
    fun testGoalValidation_RejectsGoalWhenLocalizationLost() {
        markFreeCorridor(10, 18, 10, 14)
        val wm = createWorldModel(pose = LocalPose.ORIGIN.copy(trackingState = TrackingQuality.LOST))
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)

        val result = GoalValidator.validate(goal, wm)
        assertFalse("Goal must be rejected when localization is lost", result.isValid)
        assertTrue(result.reason?.contains("lost") == true)
    }

    // --- B. A* PATH GENERATION TESTS ---
    @Test
    fun testAStarPlanner_PlansStraightPathInFreeCorridor() {
        // Create 3m forward free corridor from origin (12, 12) to (20, 12)
        markFreeCorridor(10, 22, 10, 14)
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.5f, targetY = 0.0f)

        val result = planner.planPath(wm, goal)
        assertTrue("A* must find straight path in clear corridor", result is PlanningResult.Success)

        val success = result as PlanningResult.Success
        assertTrue("Path should contain waypoints", success.path.size >= 2)
        assertEquals(0.0f, success.path.waypoints.first().xM, 0.25f)
        assertEquals(1.5f, success.path.waypoints.last().xM, 0.25f)
    }

    // --- C. BLOCKED PATH DETOUR TESTS ---
    @Test
    fun testAStarPlanner_FindsDetourAroundObstacle() {
        // Free corridor with obstacle directly in front at (1.0m, 0.0m)
        markFreeCorridor(10, 22, 8, 16)
        // Block direct forward route at X=1.0m (cell gx=16, gy=12)
        map.cells[16][12].state = MapCellState.OCCUPIED
        map.cells[16][12].confidence = 0.95f

        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 2.0f, targetY = 0.0f)

        val result = planner.planPath(wm, goal)
        assertTrue("A* must find detour around obstacle", result is PlanningResult.Success)

        val success = result as PlanningResult.Success
        // Ensure no waypoint on the path is at the obstacle location (16, 12)
        for (wp in success.path.waypoints) {
            val gx = ((wp.xM / map.resolutionM) + map.gridWidth / 2f).toInt()
            val gy = ((wp.yM / map.resolutionM) + map.gridHeight / 2f).toInt()
            assertFalse("Path waypoint must not intersect obstacle", gx == 16 && gy == 12)
        }
    }

    // --- D. UNKNOWN-CELL REJECTION TESTS ---
    @Test
    fun testAStarPlanner_RejectsTraversalThroughUnknownCells() {
        // Only origin cell is marked free, rest of map is UNCERTAIN
        map.cells[12][12].state = MapCellState.FREE
        map.cells[12][12].confidence = 0.9f
        // Goal at (1.5m, 0m) is UNCERTAIN
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.5f, targetY = 0.0f)

        val result = planner.planPath(wm, goal)
        assertTrue("A* must fail when path traverses unknown terrain", result is PlanningResult.Failure)
    }

    // --- E. ROVER FOOTPRINT COLLISION TESTS ---
    @Test
    fun testAStarPlanner_RejectsNarrowGapSmallerThanRoverFootprint() {
        // Rover footprint is 0.35m wide; in 0.25m grid, inflation requires at least 2 clear cells lateral width
        // Create narrow 1-cell corridor bounded by obstacles on both sides
        for (gx in 10..20) {
            map.cells[gx][11].state = MapCellState.OCCUPIED // obstacle left
            map.cells[gx][12].state = MapCellState.FREE     // 1 cell wide (0.25m < 0.35m + safety margin)
            map.cells[gx][13].state = MapCellState.OCCUPIED // obstacle right
        }

        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.5f, targetY = 0.0f)

        val result = planner.planPath(wm, goal)
        assertTrue("Path planner must reject narrow gap smaller than rover footprint", result is PlanningResult.Failure)
    }

    // --- F. PATH VALIDITY TESTS ---
    @Test
    fun testPathValidator_ValidatesClearPath() {
        markFreeCorridor(10, 20, 10, 14)
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f)

        val planResult = planner.planPath(wm, goal) as PlanningResult.Success
        val validation = pathValidator.validatePath(planResult.path, wm)
        assertTrue("Clear path must be valid", validation.isValid)
    }

    // --- G. PATH REPLANNING TESTS ---
    @Test
    fun testPathValidator_InvalidatesPathWhenObstacleSuddenlyAppears() {
        markFreeCorridor(10, 20, 10, 14)
        val wm = createWorldModel()
        val goal = LocalGoal(targetX = 1.5f, targetY = 0.0f)

        val planResult = planner.planPath(wm, goal) as PlanningResult.Success
        assertTrue(pathValidator.validatePath(planResult.path, wm).isValid)

        // Dynamic obstacle suddenly appears along the path at (1.0m, 0.0m) -> cell (16, 12)
        map.cells[16][12].state = MapCellState.OCCUPIED
        map.cells[16][12].confidence = 0.95f

        val updatedValidation = pathValidator.validatePath(planResult.path, wm)
        assertFalse("Path must become invalid when obstacle appears on segment", updatedValidation.isValid)
        assertTrue(updatedValidation.reason?.contains("Obstacle") == true)
    }

    // --- H. GOAL REACHED TESTS ---
    @Test
    fun testGoalDistance_DetectsArrivalWithinTolerance() {
        val goal = LocalGoal(targetX = 1.0f, targetY = 0.0f, positionToleranceM = 0.20f)
        // Position at 0.90m is 0.10m away (inside 0.20m tolerance)
        assertTrue(goal.distanceTo(0.90f, 0.0f) <= goal.positionToleranceM)
        // Position at 0.70m is 0.30m away (outside 0.20m tolerance)
        assertFalse(goal.distanceTo(0.70f, 0.0f) <= goal.positionToleranceM)
    }
}
