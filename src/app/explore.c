#include "explore.h"

#include "adc.h"
#include "controller.h"
#include "motors.h"
#include "uart.h"

#include <stdio.h>

static volatile MouseState state;

// Initialize mouse start state
void explore_init(void)
{
    state.pos.x = START_X;
    state.pos.y = START_Y;
    state.dist = 0;
    state.step_ready = true;
    state.driving_to_goal = false;
    state.step_idx = 0;
    state.total_dist_prev = 0;
    state.dir = START_DIR;
    queue_init(&state.queue);
    queue_push(&state.queue, explore_makePos(0u, 1u)); // Add cell (0,1) to queue

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
    float total_dist = getLeftDistanceMeters() * 1000.0f;

    float delta_dist = total_dist - state.total_dist_prev;
    state.total_dist_prev = total_dist;
    state.dist += delta_dist;

    unsigned int mid_dist = readMidSensorValue();
    if (mid_dist > 1500 && getDriveStatePtr()->mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
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
    if (!state.step_ready)
    {
        return;
    }
    state.step_ready = false;

    writeUART("\nSTEP\n");

    if (!state.driving_to_goal)
    {
        writeUART("EXPLORING!\n");
        if (!queue_pop(&state.queue, &state.pos))
        {
            writeUART("\n######## Finished exploring! ########\n");
            stopDriveControl();
            state.driving_to_goal = false;
            state.step_idx = 0;
            state.step_ready = false;
            return;
        }

        stopDriveControl();
    }
    else
    {
        writeUART("DRIVING TO GOAL!\n");
        state.pos = explore_getPosFromDirection(state.dir);
    }

    char buf[35];
    snprintf(buf, sizeof(buf), "Current position: X=%d Y= %d\n", state.pos.x, state.pos.y);
    writeUART(buf);

    GlobalDirection next_dir = explore_setNeighbor(REAR);

    if (!isWallLeft())
    {
        next_dir = explore_setNeighbor(LEFT);
        explore_setQueue(next_dir);
    }

    if (!isWallRight())
    {
        next_dir = explore_setNeighbor(RIGHT);
        explore_setQueue(next_dir);
    }

    if (!isWallFront())
    {
        next_dir = explore_setNeighbor(FRONT);
        explore_setQueue(next_dir);
        writeUART("NO WALL FRONT\n");
    }

    state.maze[state.pos.x][state.pos.y].explored = true;

    if (!state.driving_to_goal && queue_is_empty(&state.queue))
    {
        writeUART("\n######## Finished exploring! ########\n");
        stopDriveControl();
        state.driving_to_goal = false;
        state.step_idx = 0;
        state.step_ready = false;
        return;
    }

    if (!queue_is_empty(&state.queue))
    {
        char queue_buf[64];
        snprintf(queue_buf, sizeof(queue_buf), "Queue head: X=%d Y= %d\n",
                 state.queue.data[state.queue.head].x,
                 state.queue.data[state.queue.head].y);
        writeUART(queue_buf);
    }

    if (!state.driving_to_goal)
    {
        state.path_to_goal = explore_getPathToPos(state.queue.data[state.queue.head]);
        explore_resetSearchBools();

        if (state.path_to_goal.number_steps > 1)
        {
            state.step_idx = 0;
            state.driving_to_goal = true;
            next_dir = state.path_to_goal.directions[state.step_idx++];
        }

        writeUART("Search path: ");
        for (uint8_t i = 0; i < state.path_to_goal.number_steps; ++i)
        {
            char path_buf[10];
            snprintf(path_buf, sizeof(path_buf), "%s ",
                     explore_globalDirToString(state.path_to_goal.directions[i]));
            writeUART(path_buf);
        }
        writeUART("\n");
    }
    else
    {
        next_dir = state.path_to_goal.directions[state.step_idx++];

        if (state.step_idx >= state.path_to_goal.number_steps)
        {
            state.driving_to_goal = false;
            state.step_idx = 0;
        }
    }

    LocalDirection turn_dir = explore_getTurnDirection(state.dir, next_dir);

    char next_dir_buf[64];
    snprintf(next_dir_buf, sizeof(next_dir_buf), "Next direction: %s -> ",
             explore_globalDirToString(next_dir));
    writeUART(next_dir_buf);

    if (turn_dir == RIGHT)
    {
        writeUART("Turning Right\n");
        turnRight90();
    }
    else if (turn_dir == LEFT)
    {
        writeUART("Turning Left\n");
        turnLeft90();
    }
    else if (turn_dir == REAR)
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

    state.dir = next_dir;
}

volatile MouseState *explore_getMouseState(void)
{
    return &state;
}

Pos explore_makePos(uint8_t x, uint8_t y)
{
    return (Pos){.x = x, .y = y};
}

// Set neighbors depending on current direction and detected walls.
GlobalDirection explore_setNeighbor(LocalDirection no_wall)
{
    volatile uint8_t *neighbors = &state.maze[state.pos.x][state.pos.y].neighbors;

    if (state.dir == NORTH)
    {
        if (no_wall == FRONT)
        {
            *neighbors |= NORTH;
            return NORTH;
        }
        else if (no_wall == RIGHT)
        {
            *neighbors |= EAST;
            return EAST;
        }
        else if (no_wall == LEFT)
        {
            *neighbors |= WEST;
            return WEST;
        }

        *neighbors |= SOUTH;
        return SOUTH;
    }
    else if (state.dir == EAST)
    {
        if (no_wall == FRONT)
        {
            *neighbors |= EAST;
            return EAST;
        }
        else if (no_wall == RIGHT)
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }
        else if (no_wall == LEFT)
        {
            *neighbors |= NORTH;
            return NORTH;
        }

        *neighbors |= WEST;
        return WEST;
    }
    else if (state.dir == SOUTH)
    {
        if (no_wall == FRONT)
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }
        else if (no_wall == RIGHT)
        {
            *neighbors |= WEST;
            return WEST;
        }
        else if (no_wall == LEFT)
        {
            *neighbors |= EAST;
            return EAST;
        }

        *neighbors |= NORTH;
        return NORTH;
    }
    else if (state.dir == WEST)
    {
        if (no_wall == FRONT)
        {
            *neighbors |= WEST;
            return WEST;
        }
        else if (no_wall == RIGHT)
        {
            *neighbors |= NORTH;
            return NORTH;
        }
        else if (no_wall == LEFT)
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }

        *neighbors |= EAST;
        return EAST;
    }

    return NONE;
}

bool explore_setQueue(GlobalDirection free_dir)
{
    if (state.maze[state.pos.x][state.pos.y].explored)
    {
        return false;
    }

    Pos new_pos;

    if (free_dir == NORTH)
    {
        new_pos = explore_makePos(state.pos.x, state.pos.y + 1);
    }
    else if (free_dir == EAST)
    {
        new_pos = explore_makePos(state.pos.x + 1, state.pos.y);
    }
    else if (free_dir == SOUTH)
    {
        new_pos = explore_makePos(state.pos.x, state.pos.y - 1);
    }
    else if (free_dir == WEST)
    {
        new_pos = explore_makePos(state.pos.x - 1, state.pos.y);
    }
    else
    {
        return false;
    }

    if (!state.maze[new_pos.x][new_pos.y].explored)
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Added new pos to queue: X=%d Y= %d\n", new_pos.x, new_pos.y);
        writeUART(buf);
        return queue_push(&state.queue, new_pos);
    }

    return false;
}

Path explore_getPathToPos(Pos new_pos)
{
    Path path = {0};
    path.number_steps = explore_recursiveSearch(path.directions, 0, state.pos, new_pos);
    return path;
}

uint8_t explore_recursiveSearch(GlobalDirection *dir_ptr, uint8_t path_length, Pos start, Pos goal)
{
    Cell *cell = &state.maze[start.x][start.y];

    if (cell->searched)
    {
        return 0;
    }
    cell->searched = true;

    if (start.x == goal.x && start.y == goal.y)
    {
        return path_length;
    }

    Positions positions = explore_getPositionsFromNeighbors(start, cell->neighbors);

    for (uint8_t i = 0u; i < positions.num; ++i)
    {
        Pos neighbor_pos = positions.neighbor_positions[i];
        dir_ptr[path_length] = positions.neighboring_directions[i];
        uint8_t found_length = explore_recursiveSearch(dir_ptr, path_length + 1, neighbor_pos, goal);
        if (found_length > 0)
        {
            return found_length;
        }
    }

    return 0;
}

Positions explore_getPositionsFromNeighbors(Pos current_pos, uint8_t neighbors)
{
    Positions positions = {0};
    uint8_t idx = 0;

    if (neighbors & NORTH)
    {
        positions.neighbor_positions[idx] = explore_makePos(current_pos.x, current_pos.y + 1);
        positions.neighboring_directions[idx] = NORTH;
        idx++;
    }

    if (neighbors & EAST)
    {
        positions.neighbor_positions[idx] = explore_makePos(current_pos.x + 1, current_pos.y);
        positions.neighboring_directions[idx] = EAST;
        idx++;
    }

    if (neighbors & SOUTH)
    {
        positions.neighbor_positions[idx] = explore_makePos(current_pos.x, current_pos.y - 1);
        positions.neighboring_directions[idx] = SOUTH;
        idx++;
    }

    if (neighbors & WEST)
    {
        positions.neighbor_positions[idx] = explore_makePos(current_pos.x - 1, current_pos.y);
        positions.neighboring_directions[idx] = WEST;
        idx++;
    }

    positions.num = idx;
    return positions;
}

LocalDirection explore_getTurnDirection(GlobalDirection current_dir, GlobalDirection next_dir)
{
    if (current_dir == next_dir)
    {
        return FRONT;
    }

    if ((current_dir == NORTH && next_dir == SOUTH) ||
        (current_dir == SOUTH && next_dir == NORTH) ||
        (current_dir == EAST && next_dir == WEST) ||
        (current_dir == WEST && next_dir == EAST))
    {
        return REAR;
    }

    if ((current_dir == NORTH && next_dir == EAST) ||
        (current_dir == EAST && next_dir == SOUTH) ||
        (current_dir == SOUTH && next_dir == WEST) ||
        (current_dir == WEST && next_dir == NORTH))
    {
        return RIGHT;
    }

    if ((current_dir == NORTH && next_dir == WEST) ||
        (current_dir == EAST && next_dir == NORTH) ||
        (current_dir == SOUTH && next_dir == EAST) ||
        (current_dir == WEST && next_dir == SOUTH))
    {
        return LEFT;
    }

    return FRONT;
}

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

void explore_resetStateDistances(void)
{
    state.dist = 0;
    state.step_ready = true;
}

void explore_resetSearchBools(void)
{
    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state.maze[x][y].searched = false;
        }
    }
}

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
