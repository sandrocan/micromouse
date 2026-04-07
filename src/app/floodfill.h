#include <stdint.h>

#define MAZE_SIZE 6
#define START_X 0
#define START_Y 0
#define START_DIR NORTH

typedef enum
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} mouseDirection;

typedef struct
{
    uint8_t walls;    // bit 3: north, bit 2: east, bit 1: south, bit 0: west
    uint8_t explored; // 0: unexplored, 1: explored
} cell;

typedef struct
{
    uint8_t x;
    uint8_t y;
    mouseDirection dir;
    cell maze[MAZE_SIZE][MAZE_SIZE];
} mouseState;