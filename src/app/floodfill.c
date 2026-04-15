#include "explore.h"

floodfill_mazeFill(MouseState *statePtr)
{
    Cell *cell_ptr = &statePtr->maze[START_X][START_Y];
    if (cell_ptr->searched)
    {
        return 0;
    }
    cell_ptr->searched = true;
}