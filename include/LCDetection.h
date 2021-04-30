#ifndef RABBIT_LCD_H
#define RABBIT_LCD_H
/*
In lidar slam, an naive lcd based on the Euclidean distance is effective. 
More advanced methods such as scan context is also useful, but if the lidar point is sparse,
the result is not reliable. Becuase detecting a false loop closure is much worse than missing a true loop closure.
therefore a lot of the-state-of-the-art systems just use a naive lcd (A-LOAM, LEGO-LOAM, LIO-SAM).
In rabbit, we also implement a naive loop closure detection.
*/

#include "Frame.h"
#endif