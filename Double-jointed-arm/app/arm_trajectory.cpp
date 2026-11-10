#include "arm_trajectory.h"

bool ArmTrajectory::start(const CartesianReference& start,
                         const CartesianReference& end, float duration_s) {
  if (duration_s <= 0.f) return false;
  start_ = start;
  end_ = end;
  duration_s_ = duration_s;
  running_ = true;
  return true;
}

CartesianReference ArmTrajectory::sample(float elapsed_s) {
  CartesianReference p{};
  if (!running_) {
    p = end_;
    return p;
  }

  interpolate(start_.x, end_.x, elapsed_s, duration_s_, p.x, p.dx, p.ddx);
  interpolate(start_.y, end_.y, elapsed_s, duration_s_, p.y, p.dy, p.ddy);

  if (elapsed_s >= duration_s_) {
    p = end_;
    p.dx = p.dy = p.ddx = p.ddy = 0.f;
    running_ = false;
  }
  return p;
}

void ArmTrajectory::interpolate(float start, float end, float elapsed_s, float duration_s,
                               float& position, float& velocity,
                               float& acceleration) {
  if (duration_s <= 0.f) {
    position = end;
    velocity = 0.f;
    acceleration = 0.f;
    return;
  }

  float s = elapsed_s / duration_s;
  if (s < 0.f) s = 0.f;
  if (s > 1.f) s = 1.f;

  const float ds = end - start;
  const float s2 = s * s;
  const float s3 = s2 * s;
  const float s4 = s3 * s;
  const float s5 = s4 * s;

  position = start + ds * (10.f * s3 - 15.f * s4 + 6.f * s5);
  velocity = ds / duration_s * (30.f * s2 - 60.f * s3 + 30.f * s4);
  acceleration =
      ds / (duration_s * duration_s) * (60.f * s - 180.f * s2 + 120.f * s3);

  if (s <= 0.f) {
    position = start;
    velocity = 0.f;
    acceleration = 0.f;
  } else if (s >= 1.f) {
    position = end;
    velocity = 0.f;
    acceleration = 0.f;
  }
}

void ArmTrajectory::abort() {
  running_ = false;
}
