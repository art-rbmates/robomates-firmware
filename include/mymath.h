#ifndef MYMATH_H
#define MYMATH_H

static inline float fmap(float x, float in_min, float in_max, float out_min, float out_max, bool clamp = true) {
  float t = (x - in_min) / (in_max - in_min);
  if (clamp) {
    if (t < 0) t = 0;
    if (t > 1) t = 1;
  }
  return out_min + t * (out_max - out_min);
}

#endif