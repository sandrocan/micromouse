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
// #define WHEEL_DIAM_MM 188.5f
// #define DEGREES_PER_CELL (CELL_SIZE_MM / WHEEL_DIAM_MM) * 360.0f

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
    uint8_t number_steps;
    GlobalDirection directions[36];
} Path;

typedef struct
{
    Pos pos;
    float dist; // Distance since last cell-mid-point
    Cell maze[MAZE_SIZE][MAZE_SIZE];
    bool step_ready;
    bool driving_to_goal;
    Path path_to_goal;
    uint8_t step_idx;
    float total_dist_prev;
    GlobalDirection dir;
    volatile PosQueue queue;
} MouseState;

typedef struct
{
    Pos neighbor_positions[4];
    GlobalDirection neighboring_directions[4];
    uint8_t num;
} Positions;

void explore_init(void);
void explore_estimateCellCenter(void);
void explore_step(void);
volatile MouseState *explore_getMouseState(void);
Pos explore_makePos(uint8_t x, uint8_t y);
GlobalDirection explore_setNeighbor(LocalDirection no_wall);
bool explore_setQueue(GlobalDirection dir);
Path explore_getPathToPos(Pos new_pos);
uint8_t explore_recursiveSearch(GlobalDirection *dir_ptr, uint8_t path_length, Pos start, Pos goal);
Positions explore_getPositionsFromNeighbors(Pos current_pos, uint8_t neighbors);
LocalDirection explore_getTurnDirection(GlobalDirection current_dir, GlobalDirection next_dir);
Pos explore_getPosFromDirection(GlobalDirection dir);
void explore_resetStateDistances(void);
void explore_resetSearchBools(void);
const char *explore_globalDirToString(GlobalDirection dir);

#endif /* EXPLORE_H */
