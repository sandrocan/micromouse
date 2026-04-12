#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "controller.h"
#include "motors.h"
#include "adc.h"
#include "uart.h"
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
    uint8_t dist_to_start;
    uint8_t dist_to_goal;
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

volatile MouseState *getMouseState(void);
Pos make_pos(uint8_t x, uint8_t y);
void floodfill_init(void);
void floodfill_step(void);
void floodfill_estimate_cell_center(void);
GlobalDirection floodfill_set_neighbor(LocalDirection no_wall);
bool floodfill_set_queue(GlobalDirection dir);
Path floodfill_get_path_to_pos(Pos new_pos);
uint8_t recursive_search(GlobalDirection *dir_ptr, uint8_t path_length, Pos start, Pos goal);
Positions
get_positions_from_neighbors(Pos current_pos, uint8_t neighbors);
LocalDirection get_turn_direction(GlobalDirection current_dir, GlobalDirection next_dir);
Pos get_pos_from_direction(GlobalDirection dir);
void reset_state_dist(void);
void reset_search_bools(void);
const char *global_direction_to_string(GlobalDirection dir);

#endif /* FLOODFILL_H */
