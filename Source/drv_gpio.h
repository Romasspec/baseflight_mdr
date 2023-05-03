#pragma once

#define digitalHi(p, i)     { p->RXTX |= i; }
#define digitalLo(p, i)     { p->RXTX &=~i; }
#define digitalToggle(p, i) { p->RXTX ^= i; }
#define digitalIn(p, i)     (p->RXTX & i)

