#ifndef M5_CONTROLLER_GEOMETRY_H_
#define M5_CONTROLLER_GEOMETRY_H_

typedef struct {
  float a;
  float alpha;
} m5Accel;

typedef struct {
  float v;
  float omega;
} m5Velocity;
m5Velocity m5v_add(m5Velocity a, m5Velocity b);
m5Velocity m5v_sub(m5Velocity a, m5Velocity b);
m5Velocity m5v_mul(m5Velocity a, float t);

typedef struct {
  float x;
  float y;
  float theta;
} m5Position;
m5Position m5p_add(m5Position a, m5Position b);
m5Position m5p_sub(m5Position a, m5Position b);
m5Position m5p_mul(m5Position a, float t);

#endif
