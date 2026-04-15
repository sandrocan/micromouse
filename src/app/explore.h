#ifndef EXPLORE_H
#define EXPLORE_H

#include <stdbool.h>
#include <stdint.h>

#include "queue.h"

#define MAZE_SIZE 6
#define START_X 0
#define START_Y 0
#define START_DIR NORTH

#define CELL_SIZE_MM 180.0f
#define GOAL_X 5
#define GOAL_Y 0

typedef enum
{
    FRONT,
    RIGHT,
    REAR,
    LEFT
} LocalDirection;

typedef enum
{
    NONE = 0,
    NORTH = (1 << 3),
    EAST = (1 << 2),
    SOUTH = (1 << 1),
    WEST = (1 << 0),
} GlobalDirection;

typedef struct
{
    uint8_t neighbors; // bit 3: north, bit 2: east, bit 1: south, bit 0: west
    bool explored;
    bool searched;
} Cell;

typedef struct
{
    uint8_t stepCount;
    GlobalDirection directions[36];
} Path;

typedef struct
{
    Pos pos;    // Current position
    float dist; // Distance since last cell-mid-point
    Cell maze[MAZE_SIZE][MAZE_SIZE];
    bool stepReady; // Ready to perform an explore step (pointing straight to next enqueued cell)
    bool finalGoalActive;
    bool drivingToGoal;
    bool finishedExploring;
    Path pathToNextPos;
    Path finalPathToGoal;
    Pos finalGoal;
    uint8_t stepIndex;
    float totalDistPrev;
    GlobalDirection dir;
    volatile PosQueue queue;
} MouseState;

typedef struct
{
    Pos positions[4];
    GlobalDirection directions[4];
    uint8_t count;
} NeighborCells;

void explore_init(void);
void explore_estimateCellCenter(void);
void explore_step(void);
volatile MouseState *explore_getMouseState(void);
Pos explore_makePos(uint8_t x, uint8_t y);
GlobalDirection explore_setNeighbor(LocalDirection noWall);
bool explore_enqueueNeighborInDirection(GlobalDirection dir);
Path explore_getPathToPos(Pos newPos);
NeighborCells explore_getNeighborCells(Pos currentPos, uint8_t neighbors);
LocalDirection explore_getTurnDirection(GlobalDirection currentDir, GlobalDirection nextDir);
Pos explore_getPosFromDirection(GlobalDirection dir);
void explore_resetStateDistances(void);
const char *explore_globalDirToString(GlobalDirection dir);

#endif /* EXPLORE_H */
