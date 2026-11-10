#include "arm_kinematics.h"
#include <cmath>

namespace {
constexpr float kPi = 3.14159265358979323846f;
float unwrapNear(float raw, float ref) {
  return raw + 2.f * kPi * roundf((ref - raw) / (2.f * kPi));
}
}

CartesianReference ArmKinematics::forward(const float q[2]) {
  CartesianReference p;
  const float c0 = cosf(q[0]);
  const float s0 = sinf(q[0]);
  const float c01 = cosf(q[0] + q[1]);
  const float s01 = sinf(q[0] + q[1]);
  p.x = l0 * c0 + l1 * c01;
  p.y = l0 * s0 + l1 * s01;
  return p;
}

bool ArmKinematics::inverse(const CartesianReference& point, const float previous_q[2],
                           int elbow, float q[2]) {
  const float r2 = point.x * point.x + point.y * point.y;
  float c1 = (r2 - l0 * l0 - l1 * l1) / (2.f * l0 * l1);
  if (c1 > 1.f + 1e-6f || c1 < -1.f - 1e-6f) return false;  // 超出工作空间
  if (c1 > 1.f) c1 = 1.f;
  if (c1 < -1.f) c1 = -1.f;

  const float s1_mag = sqrtf(fmaxf(0.f, 1.f - c1 * c1));
  if (s1_mag < min_sin_q1) return false;  // 雅可比奇异

  const float s1 = (elbow >= 0) ? s1_mag : -s1_mag;
  const float q1_raw = atan2f(s1, c1);
  const float q0_raw =
      atan2f(point.y, point.x) - atan2f(l1 * s1, l0 + l1 * c1);

  q[0] = unwrapNear(q0_raw, previous_q[0]);
  q[1] = unwrapNear(q1_raw, previous_q[1]);
  return true;
}

bool ArmKinematics::lineReachable(const CartesianReference& start,
                                const CartesianReference& end) {
  constexpr int kSamples = 100;
  const float r_min = fabsf(l1 - l0);
  const float r_max = l0 + l1;
  for (int i = 0; i <= kSamples; ++i) {
    const float t = static_cast<float>(i) / kSamples;
    const float x = start.x + (end.x - start.x) * t;
    const float y = start.y + (end.y - start.y) * t;
    const float r = sqrtf(x * x + y * y);
    if (r < r_min || r > r_max) return false;
  }
  return true;
}

bool ArmKinematics::derivatives(const CartesianReference& point, const float q[2],
                               float dq[2], float ddq[2]) {
  const float s0 = sinf(q[0]);
  const float c0 = cosf(q[0]);
  const float s01 = sinf(q[0] + q[1]);
  const float c01 = cosf(q[0] + q[1]);

  const float J11 = -l0 * s0 - l1 * s01;
  const float J12 = -l1 * s01;
  const float J21 = l0 * c0 + l1 * c01;
  const float J22 = l1 * c01;

  const float det = J11 * J22 - J12 * J21;
  if (fabsf(det) < 1e-6f) return false;

  dq[0] = (J22 * point.dx - J12 * point.dy) / det;
  dq[1] = (-J21 * point.dx + J11 * point.dy) / det;

  ddq[0] = (J22 * point.ddx - J12 * point.ddy) / det;
  ddq[1] = (-J21 * point.ddx + J11 * point.ddy) / det;
  return true;
}
