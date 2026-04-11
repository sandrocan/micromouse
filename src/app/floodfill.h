#ifndef FLOODFILL_H
#define FLOODFILL_H

#include <stdbool.h>
#include <stdint.h>

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
} Cell;

typedef struct
{
    Pos pos;
    float dist; // Distance since last cell-mid-point
    Cell maze[MAZE_SIZE][MAZE_SIZE];
    bool step_ready;
    float total_dist_prev;
    GlobalDirection dir;
    volatile PosQueue queue;
} MouseState;

typedef struct
{
    uint8_t number_steps;
    GlobalDirection *directions;
} Path;

typedef struct
{
    Pos *next_positions;
    GlobalDirection *next_directions;
    uint8_t num;
} Positions;

volatile MouseState *getMouseState(void);
Pos pos_make(uint8_t x, uint8_t y);
void floodfill_init();
void floodfill_step();
void floodfill_estimate_cell_center();
GlobalDirection floodfill_set_neighbor(LocalDirection no_wall);
bool floodfill_set_queue(GlobalDirection dir);
Path floodfill_get_path_to_pos(Pos new_pos);
Positions get_positions_from_neighbors(Pos current_pos, uint8_t neighbors);
LocalDirection get_turn_direction(GlobalDirection current_dir, GlobalDirection next_dir);
void reset_state_dist(void);

#endif /* FLOODFILL_H */
