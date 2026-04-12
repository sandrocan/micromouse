#include "floodfill.h"

static volatile MouseState state;
static volatile MouseState *state_ptr = &state;

volatile MouseState *getMouseState(void)
{
    return state_ptr;
}

Pos make_pos(uint8_t x, uint8_t y)
{
    return (Pos){.x = x, .y = y};
}

// Initialize mouse start state
void floodfill_init(void)
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
    queue_push(&state.queue, make_pos(0u, 1u)); // Add cell (0,1) to queue

    // Init maze
    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state.maze[x][y].neighbors = 0;
            state.maze[x][y].dist_to_start = 255;
            state.maze[x][y].dist_to_goal = 255;
            state.maze[x][y].explored = false;
            state.maze[x][y].searched = false;
        }
    }

    // Start cell
    state.maze[START_X][START_Y].explored = true;
    // Start cell has free way to cell above
    state.maze[START_X][START_Y].neighbors |= NORTH;
    state.maze[START_X][START_Y].dist_to_goal = 0;
}

// Decides where to go next
void floodfill_step(void)
{
    if (!state.step_ready)
    {
        return;
    }
    state.step_ready = false;

    writeUART("\nSTEP\n");

    // Update mouse position and maze
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
        else
        {
            stopDriveControl();
        }
    }
    else
    {
        writeUART("DRIVING TO GOAL!\n");
        // While following a path, we arrive in the cell reached by the direction commanded in the previous step.
        state.pos = get_pos_from_direction(state.dir);
    }

    char buf[35];
    snprintf(buf, sizeof(buf), "Current position: X=%d Y= %d\n", state.pos.x, state.pos.y);
    writeUART(buf);

    // Turn around by default and set previous cell as neighbor
    GlobalDirection next_dir = floodfill_set_neighbor(REAR);

    // Set cell neighbors and exploring queue
    if (!isWallLeft())
    {
        next_dir = floodfill_set_neighbor(LEFT);
        floodfill_set_queue(next_dir);
    }

    if (!isWallRight())
    {
        next_dir = floodfill_set_neighbor(RIGHT);
        floodfill_set_queue(next_dir);
    }

    if (!isWallFront())
    {
        next_dir = floodfill_set_neighbor(FRONT);
        floodfill_set_queue(next_dir);
        writeUART("NO WALL FRONT\n");
    }

    state.maze[state.pos.x][state.pos.y].explored = true;

    // Check if new positions were added to queue. If the queue is now empty everything was explored.
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
        char buf[64];
        snprintf(buf, sizeof(buf), "Queue head: X=%d Y= %d\n", state.queue.data[state.queue.head].x, state.queue.data[state.queue.head].y);
        writeUART(buf);
    }

    if (!state.driving_to_goal)
    {
        state.path_to_goal = floodfill_get_path_to_pos(state.queue.data[state.queue.head]);
        reset_search_bools();

        if (state.path_to_goal.number_steps > 1)
        {
            state.step_idx = 0;
            state.driving_to_goal = true;
            next_dir = state.path_to_goal.directions[state.step_idx++];
        }

        writeUART("Search path: ");
        for (uint8_t i = 0; i < state.path_to_goal.number_steps; ++i)
        {
            char buf3[10];
            snprintf(buf3, sizeof(buf3), "%s ", global_direction_to_string(state.path_to_goal.directions[i]));
            writeUART(buf3);
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

    LocalDirection turn_dir = get_turn_direction(state.dir, next_dir);

    char buf2[64];
    snprintf(buf2, sizeof(buf2), "Next direction: %s -> ", global_direction_to_string(next_dir));
    writeUART(buf2);

    // After turning the mouse drives straight automatically
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
        reset_state_dist();
        driveStraight();
        writeUART("Driving straight\n");
    }

    state.dir = next_dir;
}

// Try to estimate if the mouse has reached the center of a cell
void floodfill_estimate_cell_center(void)
{
    float total_dist = getLeftDistanceMeters() * 1000.0f;

    float delta_dist = total_dist - state.total_dist_prev;
    state.total_dist_prev = total_dist;
    state.dist += delta_dist;

    unsigned int mid_dist = readMidSensorValue();
    // Robot reached estimated cell center (via sensor reading)
    if (mid_dist > 1500 && getDriveStatePtr()->mode == CONTROLLER_MODE_DRIVE_STRAIGHT)
    {
        writeUART("Cell center detected - SENSOR\n\n");
        floodfill_step();
        return;
    }

    // Robot reached estimated cell center (via distance estimation)
    if (state.dist >= CELL_SIZE_MM)
    {
        // writeUART("Cell center detected - DISTANCE\n\n");
        floodfill_step();
        return;
    }
}

// Set neighbors depending on current direction and detected walls. Returns NONE if current cell is already explored.
GlobalDirection floodfill_set_neighbor(LocalDirection no_wall)
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
        else
        {
            *neighbors |= SOUTH;
            return SOUTH;
        }
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
        else
        {
            *neighbors |= WEST;
            return WEST;
        }
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
        else
        {
            *neighbors |= NORTH;
            return NORTH;
        }
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
    if (state.maze[state.pos.x][state.pos.y].explored)
    {
        return false;
    }

    Pos new_pos;

    if (free_dir == NORTH)
    {
        new_pos = make_pos(state.pos.x, state.pos.y + 1);
    }
    else if (free_dir == EAST)
    {
        new_pos = make_pos(state.pos.x + 1, state.pos.y);
    }
    else if (free_dir == SOUTH)
    {
        new_pos = make_pos(state.pos.x, state.pos.y - 1);
    }
    else if (free_dir == WEST)
    {
        new_pos = make_pos(state.pos.x - 1, state.pos.y);
    }
    else
    {
        return false;
    }

    // Only add new position to queue if it is not explored
    if (!state.maze[new_pos.x][new_pos.y].explored)
    {
        char buf[40];
        snprintf(buf, sizeof(buf), "Added new pos to queue: X=%d Y= %d\n", new_pos.x, new_pos.y);
        writeUART(buf);
        return queue_push(&state.queue, new_pos);
    }

    return false;
}

Path floodfill_get_path_to_pos(Pos new_pos)
{
    Path path = {0};
    path.number_steps = recursive_search(path.directions, 0, state.pos, new_pos);
    return path;
}

uint8_t recursive_search(GlobalDirection *dir_ptr, uint8_t path_length, Pos start, Pos goal)
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

    Positions positions = get_positions_from_neighbors(start, cell->neighbors);

    // Depth-first search with backtracking. As soon as a branch reaches the
    // goal, the written directions in dir_ptr form the path prefix to it.
    for (uint8_t i = 0u; i < positions.num; ++i)
    {
        Pos neighbor_pos = positions.neighbor_positions[i];
        dir_ptr[path_length] = positions.neighboring_directions[i];
        uint8_t found_length = recursive_search(dir_ptr, path_length + 1, neighbor_pos, goal);
        if (found_length > 0)
        {
            return found_length;
        }
    }

    return 0;
}

Positions get_positions_from_neighbors(Pos current_pos, uint8_t neighbors)
{
    Positions positions = {0};
    uint8_t idx = 0;

    if (neighbors & NORTH)
    {
        positions.neighbor_positions[idx] = make_pos(current_pos.x, current_pos.y + 1);
        positions.neighboring_directions[idx] = NORTH;
        idx++;
    }

    if (neighbors & EAST)
    {
        positions.neighbor_positions[idx] = make_pos(current_pos.x + 1, current_pos.y);
        positions.neighboring_directions[idx] = EAST;
        idx++;
    }

    if (neighbors & SOUTH)
    {
        positions.neighbor_positions[idx] = make_pos(current_pos.x, current_pos.y - 1);
        positions.neighboring_directions[idx] = SOUTH;
        idx++;
    }

    if (neighbors & WEST)
    {
        positions.neighbor_positions[idx] = make_pos(current_pos.x - 1, current_pos.y);
        positions.neighboring_directions[idx] = WEST;
        idx++;
    }

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

    if ((current_dir == NORTH && next_dir == WEST) ||
        (current_dir == EAST && next_dir == NORTH) ||
        (current_dir == SOUTH && next_dir == EAST) ||
        (current_dir == WEST && next_dir == SOUTH))
    {
        return LEFT;
    }

    return FRONT;
}

Pos get_pos_from_direction(GlobalDirection dir)
{
    if (dir == NORTH)
    {
        return make_pos(state.pos.x, state.pos.y + 1);
    }
    else if (dir == EAST)
    {
        return make_pos(state.pos.x + 1, state.pos.y);
    }
    else if (dir == SOUTH)
    {
        return make_pos(state.pos.x, state.pos.y - 1);
    }
    else if (dir == WEST)
    {
        return make_pos(state.pos.x - 1, state.pos.y);
    }

    return state.pos;
}

void reset_state_dist(void)
{
    state.dist = 0;
    state.step_ready = true;
}

void reset_search_bools(void)
{
    for (uint8_t x = 0; x < MAZE_SIZE; x++)
    {
        for (uint8_t y = 0; y < MAZE_SIZE; y++)
        {
            state.maze[x][y].searched = false;
        }
    }
}

const char *global_direction_to_string(GlobalDirection dir)
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
