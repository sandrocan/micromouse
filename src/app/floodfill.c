#include "floodfill.h"

void init_floodfill(mouseState *state)
{
    state->x = START_X;
    state->y = START_Y;
    state->dir = START_DIR;

    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state->maze[x][y].walls = 0;
            state->maze[x][y].explored = 0;
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

    state->maze[START_X][START_Y].explored = 1;
    state->maze[START_X][START_Y].walls |= (1 << 2); // Start cell has an east wall (left and bottem walls already set by outer wall initialization)
}
