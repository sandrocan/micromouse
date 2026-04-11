#include "floodfill.h"

static volatile MouseState m;
static volatile MouseState *state = &m;

volatile MouseState *getMouseState(void)
{
    return state;
}

Pos pos_make(uint8_t x, uint8_t y)
{
    return (Pos){.x = x, .y = y};
}

// Initialize mouse start state
void floodfill_init(void)
{
    state->pos.x = START_X;
    state->pos.y = START_Y;
    state->dist = 0;
    state->step_ready = true;
    state->total_dist_prev = 0;
    state->dir = START_DIR;
    queue_init(&state->queue);
    queue_push(&state->queue, pos_make(0u, 1u)); // Add cell (0,1) to queue

    // Init maze
    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state->maze[x][y].neighbors = 0;
            state->maze[x][y].dist_to_start = 255;
            state->maze[x][y].dist_to_goal = 255;
            state->maze[x][y].explored = false;
        }
    }

    // Start cell
    state->maze[START_X][START_Y].explored = true;
    // Start cell has free way to cell above
    state->maze[START_X][START_Y].neighbors |= NORTH;
    state->maze[START_X][START_Y].dist_to_goal = 0;
}

// Decides where to go next
void floodfill_step(void)
{
    if (!state->step_ready)
    {
        return;
    }
    state->step_ready = false;

    writeUART("STEP\n");

    stopDriveControl();

    // Update mouse position and maze
    if (queue_pop(&state->queue, &state->pos))
    {
        // Position is here already updated
        state->maze[state->pos.x][state->pos.y].explored = true;
        writeUART("POS updated\n");
    }
    else
    {
        // TODO: queue empty => Exploring finished!
    }

    // Turn around by default and set previous cell as neighbor
    GlobalDirection next_dir = floodfill_set_neighbor(REAR);

    // Set cell neighbors and exploring queue
    if (!isWallLeft())
    {
        next_dir = floodfill_set_neighbor(LEFT);
        floodfill_set_queue(next_dir);
        writeUART("NO WALL LEFT\n");
    }

    if (!isWallRight())
    {
        next_dir = floodfill_set_neighbor(RIGHT);
        floodfill_set_queue(next_dir);
        writeUART("NO WALL RIGHT\n");
    }

    if (!isWallFront())
    {
        next_dir = floodfill_set_neighbor(FRONT);
        floodfill_set_queue(next_dir);
        writeUART("NO WALL FRONT\n");
    }

    char buf[64];
    snprintf(buf, sizeof(buf), "Queue head= X=%d Y= %d\n", state->queue.data[state->queue.head].x, state->queue.data[state->queue.head].y);
    writeUART(buf);

    Path path = floodfill_get_path_to_pos(state->queue.data[state->queue.head]);

    LocalDirection turn_dir = get_turn_direction(state->dir, path.directions[0]);

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
    else {
        reset_state_dist();
    }

    state->dir = next_dir;
    writeUART("Driving straight\n");
    driveStraight();
}

// Try to estimate if the mouse has reached the center of a cell
void floodfill_estimate_cell_center(void)
{
    float total_dist = getLeftDistanceMeters() * 1000.0f;

    float delta_dist = total_dist - state->total_dist_prev;
    state->total_dist_prev = total_dist;
    state->dist += delta_dist;

    unsigned int mid_dist = readMidSensorValue();
    // Robot reached estimated cell center (via sensor reading)
    if (mid_dist < 1300 && mid_dist > 1100 && getDriveStatePtr()->mode == CONTROLLER_MODE_DRIVE_STRAIGHT) 
    {
        writeUART("MIDDLE BY SENSOR\n");
        floodfill_step();
        return;
    }

    // Robot reached estimated cell center (via distance estimation)
    if (state->dist >= CELL_SIZE_MM)
    {
        writeUART("MIDDLE BY DISTANCE\n");
        floodfill_step();
        return;
    }
}

// Set neibors depending on current direction and detected walls
GlobalDirection floodfill_set_neighbor(LocalDirection no_wall)
{
    volatile uint8_t *neighbors = &state->maze[state->pos.x][state->pos.y].neighbors;

    if (state->dir == NORTH)
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
        else
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }
    }
    else if (state->dir == EAST)
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
        else
        {
            *neighbors |= WEST;
            return WEST;
        }
    }
    else if (state->dir == SOUTH)
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
        else
        {
            *neighbors |= NORTH;
            return NORTH;
        }
    }
    else if (state->dir == WEST)
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
        else
        {
            *neighbors |= EAST;
            return EAST;
        }
    }

    return NONE;
}

bool floodfill_set_queue(GlobalDirection free_dir)
{
    Pos new_pos;

    if (free_dir == NORTH)
    {
        new_pos = pos_make(state->pos.x, state->pos.y + 1);
    }
    else if (free_dir == EAST)
    {
        new_pos = pos_make(state->pos.x + 1, state->pos.y);
    }
    else if (free_dir == SOUTH)
    {
        new_pos = pos_make(state->pos.x, state->pos.y - 1);
    }
    else
    {
        new_pos = pos_make(state->pos.x - 1, state->pos.y);
    }

    // Only add new position to queue if it is not explored
    if (!state->maze[new_pos.x][new_pos.y].explored)
    {
        return queue_push(&state->queue, new_pos);
    }

    return false;
}

Path floodfill_get_path_to_pos(Pos new_pos)
{
    Path path = {0, NONE};
    Pos pos = state->pos;

    // Current position is new position
    if (new_pos.x == pos.x && new_pos.y == pos.y)
    {
        return path;
    }

    Positions positions = get_positions_from_neighbors(pos, state->maze[pos.x][pos.y].neighbors);

    for (uint8_t i = 0u; i < positions.num; ++i)
    {
        Pos temp_pos = positions.next_positions[i];
        if (temp_pos.x == new_pos.x && temp_pos.y == new_pos.y)
        {
            path.number_steps = 1;
            path.directions = &positions.next_directions[i];
            return path;
        }
    }

    return path;
}

Positions get_positions_from_neighbors(Pos current_pos, uint8_t neighbors)
{
    Pos pos_arr[4];
    GlobalDirection dir_arr[4];
    uint8_t idx = 0;

    if (neighbors & NORTH)
    {
        pos_arr[idx] = pos_make(current_pos.x, current_pos.y + 1);
        dir_arr[idx] = NORTH;
        idx++;
    }

    if (neighbors & EAST)
    {
        pos_arr[idx] = pos_make(current_pos.x + 1, current_pos.y);
        dir_arr[idx] = EAST;
        idx++;
    }

    if (neighbors & SOUTH)
    {
        pos_arr[idx] = pos_make(current_pos.x, current_pos.y - 1);
        dir_arr[idx] = SOUTH;
        idx++;
    }

    if (neighbors & WEST)
    {
        pos_arr[idx] = pos_make(current_pos.x - 1, current_pos.y);
        dir_arr[idx] = WEST;
        idx++;
    }

    Positions positions;
    positions.next_positions = pos_arr;
    positions.next_directions = dir_arr;
    positions.num = idx;

    return positions;
}

LocalDirection get_turn_direction(GlobalDirection current_dir, GlobalDirection next_dir)
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

    return LEFT;
}

void reset_state_dist(void)
{
    state->dist = 0;
    state->step_ready = true;
}
