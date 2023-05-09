#include "board.h"
#include "mw.h"

static bool rxMspFrameDone = false;

void mspFrameRecieve(void)
{
    rxMspFrameDone = true;
}
