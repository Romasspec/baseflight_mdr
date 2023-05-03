#include "board.h"

void serialWrite(serialPort_t *instance, uint8_t ch)
{
    instance->vTable->serialWrite(instance, ch);
}

