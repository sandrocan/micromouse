#include "explore.h"

#include "adc.h"
#include "controller.h"
#include "motors.h"
#include "timers.h"
#include "uart.h"

#include <stdio.h>
#include <limits.h>

static volatile MouseState state;

static void explore_clearSearchFlags(void);
static void explore_recursiveSearchBest(GlobalDirection *currentPath,
                                        uint8_t currPathLength,
                                        Pos start,
                                        Pos goal,
                                        GlobalDirection *bestPath,
                                        uint8_t *bestLength);

// Initialize mouse start state
void explore_init(void)
{
    state.pos.x = START_X;
    state.pos.y = START_Y;
    state.dist = 0;
    state.stepReady = true;
    state.finalGoalActive = false;
    state.drivingToGoal = false;
    state.finishedExploring = false;
    state.stepIndex = 0;
    state.totalDistPrev = 0;
    state.dir = START_DIR;
    state.finalGoal = explore_makePos(GOAL_X, GOAL_Y);
    queue_init(&state.queue);                        // Initialize LIFO queue
    queue_push(&state.queue, explore_makePos(0, 0)); // The robot should return zu its start position if everything else is explored
    queue_push(&state.queue, explore_makePos(0, 1)); // Add cell (0,1) to queue -> First point where to go

    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state.maze[x][y].neighbors = 0;
            state.maze[x][y].explored = false;
            state.maze[x][y].searched = false;
        }
    }

    state.maze[START_X][START_Y].explored = true;
    state.maze[START_X][START_Y].neighbors |= NORTH;
}

// Try to estimate if the mouse has reached the center of a cell
void explore_estimateCellCenter(void)
{
    float totalDist = getLeftDistanceMeters() * 1000.0f;

    float deltaDist = totalDist - state.totalDistPrev;
    state.totalDistPrev = totalDist;
    state.dist += deltaDist;

    unsigned int midDist = readMidSensorValue();
    if (midDist > 1500 && getDriveStatePtr()->mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    {
        writeUART("Cell center detected - SENSOR\n\n");
        explore_step();
        return;
    }

    if (state.dist >= CELL_SIZE_MM)
    {
        explore_step();
        return;
    }
}

// Decides where to go next
void explore_step(void)
{
    if (!state.stepReady)
    {
        return;
    }
    state.stepReady = false;

    writeUART("\nSTEP\n");

    if (state.finalGoalActive)
    {
        writeUART("DRIVING TO FINAL GOAL!\n");
        if (state.drivingToGoal || state.finalPathToGoal.stepCount > 0)
        {
            state.pos = explore_getPosFromDirection(state.dir);
        }
    }
    else if (!state.drivingToGoal)
    {
        writeUART("EXPLORING!\n");
        if (!queue_pop(&state.queue, &state.pos))
        {
            writeUART("\n######## Finished exploring! ########\n");
            state.finishedExploring = true;
            if (state.dir == NORTH)
            {
                state.dir = SOUTH;
            }
            else if (state.dir == EAST)
            {
                state.dir = WEST;
            }
            else if (state.dir == SOUTH)
            {
                state.dir = NORTH;
            }
            else if (state.dir == WEST)
            {
                state.dir = EAST;
            }
            turn180();
            state.drivingToGoal = false;
            state.stepIndex = 0;
            state.stepReady = false;
            return;
        }
    }
    else
    {
        writeUART("DRIVING TO GOAL!\n");
        state.pos = explore_getPosFromDirection(state.dir);
    }

    char buf[35];
    snprintf(buf, sizeof(buf), "Current position: X=%d Y= %d\n", state.pos.x, state.pos.y);
    writeUART(buf);

    GlobalDirection nextDir = explore_setNeighbor(REAR);

    if (!isWallLeft())
    {
        nextDir = explore_setNeighbor(LEFT);
        explore_enqueueNeighborInDirection(nextDir);
    }

    if (!isWallRight())
    {
        nextDir = explore_setNeighbor(RIGHT);
        explore_enqueueNeighborInDirection(nextDir);
    }

    if (!isWallFront())
    {
        nextDir = explore_setNeighbor(FRONT);
        explore_enqueueNeighborInDirection(nextDir);
        writeUART("NO WALL FRONT\n");
    }

    state.maze[state.pos.x][state.pos.y].explored = true;

    if (!state.finalGoalActive && !state.drivingToGoal && queue_is_empty(&state.queue))
    {
        writeUART("\n######## Finished exploring! ########\n");
        state.finishedExploring = true;
        if (state.dir == NORTH)
        {
            state.dir = SOUTH;
        }
        else if (state.dir == EAST)
        {
            state.dir = WEST;
        }
        else if (state.dir == SOUTH)
        {
            state.dir = NORTH;
        }
        else if (state.dir == WEST)
        {
            state.dir = EAST;
        }
        turn180();
        state.drivingToGoal = false;
        state.stepIndex = 0;
        state.stepReady = false;
        return;
    }

    if (!state.finalGoalActive && !queue_is_empty(&state.queue))
    {
        char queue_buf[64];
        snprintf(queue_buf, sizeof(queue_buf), "Queue head: X=%d Y= %d\n",
                 state.queue.data[state.queue.head].x,
                 state.queue.data[state.queue.head].y);
        writeUART(queue_buf);
    }

    if (state.finalGoalActive)
    {
        if (!state.drivingToGoal)
        {
            state.stepIndex = 0;
            state.finalPathToGoal = explore_getPathToPos(state.finalGoal);

            writeUART("Final path: ");
            for (uint8_t i = 0; i < state.finalPathToGoal.stepCount; ++i)
            {
                char pathBuf[10];
                snprintf(pathBuf, sizeof(pathBuf), "%s ",
                         explore_globalDirToString(state.finalPathToGoal.directions[i]));
                writeUART(pathBuf);
            }
            writeUART("\n");

            if (state.finalPathToGoal.stepCount == 0)
            {
                if (state.pos.x == state.finalGoal.x && state.pos.y == state.finalGoal.y)
                {
                    writeUART("\n######## Final goal reached! ########\n");
                    startGoalMelody();
                }
                else
                {
                    writeUART("\n######## No path to final goal! ########\n");
                }

                state.finalGoalActive = false;
                state.drivingToGoal = false;
                state.stepIndex = 0;
                state.dist = 0;
                state.stepReady = false;
                stopDriveControl();
                return;
            }

            if (state.finalPathToGoal.stepCount > 1)
            {
                state.drivingToGoal = true;
            }

            nextDir = state.finalPathToGoal.directions[state.stepIndex++];
        }
        else
        {
            nextDir = state.finalPathToGoal.directions[state.stepIndex++];

            if (state.stepIndex >= state.finalPathToGoal.stepCount)
            {
                state.drivingToGoal = false;
                state.stepIndex = 0;
            }
        }
    }
    else
    {
        // When not already following a multi-step path, compute the path to the next queued cell.
        if (!state.drivingToGoal)
        {
            state.pathToNextPos = explore_getPathToPos(state.queue.data[state.queue.head]);
            nextDir = state.pathToNextPos.directions[0];

            // Only switch into path-following mode if the next queued cell is not directly adjacent.
            if (state.pathToNextPos.stepCount > 1)
            {
                state.stepIndex = 1;
                state.drivingToGoal = true;
            }

            writeUART("Search path: ");
            for (uint8_t i = 0; i < state.pathToNextPos.stepCount; ++i)
            {
                char pathBuf[10];
                snprintf(pathBuf, sizeof(pathBuf), "%s ",
                         explore_globalDirToString(state.pathToNextPos.directions[i]));
                writeUART(pathBuf);
            }
            writeUART("\n");
        }
        else
        {
            // Continue following the current multi-step path.
            nextDir = state.pathToNextPos.directions[state.stepIndex++];

            if (state.stepIndex >= state.pathToNextPos.stepCount)
            {
                state.drivingToGoal = false;
                state.stepIndex = 0;
            }
        }
    }

    LocalDirection turnDir = explore_getTurnDirection(state.dir, nextDir);

    char nextDirBuf[64];
    snprintf(nextDirBuf, sizeof(nextDirBuf), "Next direction: %s -> ",
             explore_globalDirToString(nextDir));
    writeUART(nextDirBuf);

    if (turnDir == RIGHT)
    {
        writeUART("Turning Right\n");
        turnRight90();
    }
    else if (turnDir == LEFT)
    {
        writeUART("Turning Left\n");
        turnLeft90();
    }
    else if (turnDir == REAR)
    {
        writeUART("Turning 180\n");
        turn180();
    }
    else
    {
        explore_resetStateDistances();
        driveStraight();
        writeUART("Driving straight\n");
    }

    state.dir = nextDir;
}

volatile MouseState *explore_getMouseState(void)
{
    return &state;
}

Pos explore_makePos(uint8_t x, uint8_t y)
{
    return (Pos){.x = x, .y = y};
}

// Set neighbors for current cell depending on direction and detected walls
GlobalDirection explore_setNeighbor(LocalDirection noWall)
{
    volatile uint8_t *neighbors = &state.maze[state.pos.x][state.pos.y].neighbors;

    if (state.dir == NORTH)
    {
        if (noWall == FRONT)
        {
            *neighbors |= NORTH;
            return NORTH;
        }
        else if (noWall == RIGHT)
        {
            *neighbors |= EAST;
            return EAST;
        }
        else if (noWall == LEFT)
        {
            *neighbors |= WEST;
            return WEST;
        }

        *neighbors |= SOUTH;
        return SOUTH;
    }
    else if (state.dir == EAST)
    {
        if (noWall == FRONT)
        {
            *neighbors |= EAST;
            return EAST;
        }
        else if (noWall == RIGHT)
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }
        else if (noWall == LEFT)
        {
            *neighbors |= NORTH;
            return NORTH;
        }

        *neighbors |= WEST;
        return WEST;
    }
    else if (state.dir == SOUTH)
    {
        if (noWall == FRONT)
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }
        else if (noWall == RIGHT)
        {
            *neighbors |= WEST;
            return WEST;
        }
        else if (noWall == LEFT)
        {
            *neighbors |= EAST;
            return EAST;
        }

        *neighbors |= NORTH;
        return NORTH;
    }
    else if (state.dir == WEST)
    {
        if (noWall == FRONT)
        {
            *neighbors |= WEST;
            return WEST;
        }
        else if (noWall == RIGHT)
        {
            *neighbors |= NORTH;
            return NORTH;
        }
        else if (noWall == LEFT)
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }

        *neighbors |= EAST;
        return EAST;
    }

    return NONE;
}

// Enqueue a neighboring reachable cell next to the current cell.
bool explore_enqueueNeighborInDirection(GlobalDirection freeDir)
{
    if (state.maze[state.pos.x][state.pos.y].explored)
    {
        return false;
    }

    Pos newPos;

    if (freeDir == NORTH)
    {
        newPos = explore_makePos(state.pos.x, state.pos.y + 1);
    }
    else if (freeDir == EAST)
    {
        newPos = explore_makePos(state.pos.x + 1, state.pos.y);
    }
    else if (freeDir == SOUTH)
    {
        newPos = explore_makePos(state.pos.x, state.pos.y - 1);
    }
    else if (freeDir == WEST)
    {
        newPos = explore_makePos(state.pos.x - 1, state.pos.y);
    }
    else
    {
        return false;
    }

    if (!queue_contains(&state.queue, newPos) &&
        !state.maze[newPos.x][newPos.y].explored)
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Added new pos to queue: X=%d Y= %d\n", newPos.x, newPos.y);
        writeUART(buf);
        return queue_push(&state.queue, newPos);
    }

    return false;
}

// Returns the path consisting of GlobalDirections from the current pos to a given new pos
Path explore_getPathToPos(Pos newPos)
{
    Path path = {0};
    uint8_t bestLength = UINT8_MAX;
    GlobalDirection currentPath[MAZE_SIZE * MAZE_SIZE] = {NONE};

    explore_clearSearchFlags();
    explore_recursiveSearchBest(currentPath, 0, state.pos, newPos, path.directions, &bestLength);
    explore_clearSearchFlags();

    if (bestLength != UINT8_MAX)
    {
        path.stepCount = bestLength;
    }

    return path;
}

static void explore_clearSearchFlags(void)
{
    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state.maze[x][y].searched = false;
        }
    }
}

static void explore_recursiveSearchBest(GlobalDirection *currentPath,
                                        uint8_t currPathLength,
                                        Pos start,
                                        Pos goal,
                                        GlobalDirection *bestPath,
                                        uint8_t *bestLength)
{
    if (currPathLength >= *bestLength || currPathLength >= (MAZE_SIZE * MAZE_SIZE))
    {
        return;
    }

    if (start.x == goal.x && start.y == goal.y)
    {
        *bestLength = currPathLength;
        for (uint8_t i = 0; i < currPathLength; ++i)
        {
            bestPath[i] = currentPath[i];
        }
        return;
    }

    Cell *cell = &state.maze[start.x][start.y];
    if (cell->searched)
    {
        return;
    }

    cell->searched = true;

    NeighborCells neighborCells = explore_getNeighborCells(start, cell->neighbors);
    for (uint8_t i = 0u; i < neighborCells.count; ++i)
    {
        currentPath[currPathLength] = neighborCells.directions[i];
        explore_recursiveSearchBest(currentPath,
                                    currPathLength + 1,
                                    neighborCells.positions[i],
                                    goal,
                                    bestPath,
                                    bestLength);
    }

    cell->searched = false;
}

NeighborCells explore_getNeighborCells(Pos currentPos, uint8_t neighbors)
{
    NeighborCells neighborCells = {{{0}}, {0}, 0};
    uint8_t idx = 0;

    if (neighbors & NORTH)
    {
        neighborCells.positions[idx] = explore_makePos(currentPos.x, currentPos.y + 1);
        neighborCells.directions[idx] = NORTH;
        idx++;
    }

    if (neighbors & EAST)
    {
        neighborCells.positions[idx] = explore_makePos(currentPos.x + 1, currentPos.y);
        neighborCells.directions[idx] = EAST;
        idx++;
    }

    if (neighbors & SOUTH)
    {
        neighborCells.positions[idx] = explore_makePos(currentPos.x, currentPos.y - 1);
        neighborCells.directions[idx] = SOUTH;
        idx++;
    }

    if (neighbors & WEST)
    {
        neighborCells.positions[idx] = explore_makePos(currentPos.x - 1, currentPos.y);
        neighborCells.directions[idx] = WEST;
        idx++;
    }

    neighborCells.count = idx;
    return neighborCells;
}

// Returns the direction the robot should turn in (FRONT is no turn)
LocalDirection explore_getTurnDirection(GlobalDirection currentDir, GlobalDirection nextDir)
{
    if (currentDir == nextDir)
    {
        return FRONT;
    }

    if ((currentDir == NORTH && nextDir == SOUTH) ||
        (currentDir == SOUTH && nextDir == NORTH) ||
        (currentDir == EAST && nextDir == WEST) ||
        (currentDir == WEST && nextDir == EAST))
    {
        return REAR;
    }

    if ((currentDir == NORTH && nextDir == EAST) ||
        (currentDir == EAST && nextDir == SOUTH) ||
        (currentDir == SOUTH && nextDir == WEST) ||
        (currentDir == WEST && nextDir == NORTH))
    {
        return RIGHT;
    }

    if ((currentDir == NORTH && nextDir == WEST) ||
        (currentDir == EAST && nextDir == NORTH) ||
        (currentDir == SOUTH && nextDir == EAST) ||
        (currentDir == WEST && nextDir == SOUTH))
    {
        return LEFT;
    }

    return FRONT;
}

// Returns the new position for a direction based on the current position
// Only called when the mouse is not getting its next neighboring position from the queue
Pos explore_getPosFromDirection(GlobalDirection dir)
{
    if (dir == NORTH)
    {
        return explore_makePos(state.pos.x, state.pos.y + 1);
    }
    else if (dir == EAST)
    {
        return explore_makePos(state.pos.x + 1, state.pos.y);
    }
    else if (dir == SOUTH)
    {
        return explore_makePos(state.pos.x, state.pos.y - 1);
    }
    else if (dir == WEST)
    {
        return explore_makePos(state.pos.x - 1, state.pos.y);
    }

    return state.pos;
}

// Called from the controller when a turn is done
void explore_resetStateDistances(void)
{
    // WHO DOWS IT CALL WHEN THE MOUSE HAS TO DO NO TURN
    state.dist = 0;
    state.stepReady = true;
}

// Used for UART output
const char *explore_globalDirToString(GlobalDirection dir)
{
    switch (dir)
    {
    case NORTH:
        return "NORTH";
    case EAST:
        return "EAST";
    case SOUTH:
        return "SOUTH";
    case WEST:
        return "WEST";
    case NONE:
        return "NONE";
    default:
        return "UNKNOWN";
    }
}
