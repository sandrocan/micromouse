#include "floodfill.h"

void init_floodfill(mouseState *state)
{
    state->x = 0;
    state->y = 0;
    state->dir = NORTH;

    for (uint8_t i = 0; i < MAZE_SIZE; i++)
    {
        for (uint8_t j = 0; j < MAZE_SIZE; j++)
        {
            state->maze[i][j].walls = 0;
            state->maze[i][j].explored = 0;
        }
    }

    // Set outer walls of the maze
    for (uint8_t i = 0; i < MAZE_SIZE; i++)
    {
        state->maze[i][MAZE_SIZE - 1].walls |= (1 << 3); // North wall
        state->maze[MAZE_SIZE - 1][i].walls |= (1 << 2); // East wall
        state->maze[i][0].walls |= (1 << 1);             // South wall
        state->maze[0][i].walls |= (1 << 0);             // West wall
    }
}