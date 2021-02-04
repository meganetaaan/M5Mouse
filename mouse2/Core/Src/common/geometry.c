#include "common/geometry.h"

m5Velocity m5v_add(m5Velocity a, m5Velocity b) {
  return (m5Velocity){a.v + b.v, a.omega + b.omega};
}
m5Velocity m5v_sub(m5Velocity a, m5Velocity b) {
  return (m5Velocity){a.v - b.v, a.omega - b.omega};
}
m5Velocity m5v_mul(m5Velocity a, float t) {
  return (m5Velocity){a.v * t, a.omega * t};
}
m5Position m5p_add(m5Position a, m5Position b) {
  return (m5Position){a.x + b.x, a.y + b.y, a.theta + b.theta};
}
m5Position m5p_sub(m5Position a, m5Position b) {
  return (m5Position){a.x - b.x, a.y - b.y, a.theta - b.theta};
}
m5Position m5p_mul(m5Position a, float t) {
  return (m5Position){a.x * t, a.y * t, a.theta * t};
}
