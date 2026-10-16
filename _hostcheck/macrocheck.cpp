
// simulate the Arduino core's abs macro to catch any remaining std::abs-style collisions
#define abs(x) ((x)>0?(x):-(x))
#define round(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
// the sketch uses: fabsf, labs, min, max (redefined by Arduino), millis etc.
float test = fabsf(-1.5f);
long l = labs(-7L);
int m = min(3, 5);
int M = max(3, 5);
int main() { return (test == 1.5f && l == 7 && m == 3 && M == 5) ? 0 : 1; }
