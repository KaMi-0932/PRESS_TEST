#ifndef NOTCH50_H
#define NOTCH50_H

/* 50 Hz biquad notch, Fs=1000 Hz, Q=10 (about 5 Hz rejection bandwidth).
 * RBJ normalized coefficients; unity DC gain. No run-time trig required.
 * Keep the 1 kHz call rate fixed when using these coefficients.
 */
#define NOTCH50_B0 0.9847842466f
#define NOTCH50_B1 -1.8731709497f
#define NOTCH50_B2 NOTCH50_B0
#define NOTCH50_A1 NOTCH50_B1
#define NOTCH50_A2 0.9695684932f

typedef struct {
  float x1, x2, y1, y2;
  unsigned char initialized;
} Notch50State;

static inline float Notch50_Process(Notch50State *s, float x)
{
  if (!s->initialized)
  {
    /* Seed at the first input level instead of introducing a DC step. */
    s->x1 = s->x2 = s->y1 = s->y2 = x;
    s->initialized = 1;
    return x;
  }
  float y = NOTCH50_B0*x + NOTCH50_B1*s->x1 + NOTCH50_B2*s->x2
            - NOTCH50_A1*s->y1 - NOTCH50_A2*s->y2;
  s->x2 = s->x1;
  s->x1 = x;
  s->y2 = s->y1;
  s->y1 = y;
  return y;
}

#endif
