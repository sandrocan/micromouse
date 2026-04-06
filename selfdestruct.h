#ifndef SELFDESTRUCT_H
#define SELFDESTRUCT_H

#include <xc.h>

void initSelfDestructInterrupt(void);
unsigned int selfDestructWasPressed(void);
void clearSelfDestructPressFlag(void);
unsigned long getSelfDestructPressCount(void);

#endif /* SELFDESTRUCT_H */
