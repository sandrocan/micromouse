#include <stdint.h>

#define MAZE_SIZE 6

typedef enum
{
    NORTH = 0,
    EAST = 1,
    SOUTH = 2,
    WEST = 3
} mouseDirection;

typedef struct
{
    uint8_t walls;    // bit 0: north, bit 1: east, bit 2: south, bit 3: west
    uint8_t explored; // 0: unexplored, 1: explored
} cell;

typedef struct
{
    uint8_t x;
    uint8_t y;
    mouseDirection dir;
    cell maze[MAZE_SIZE][MAZE_SIZE];
} mouseState;