//
// Created by NoSignal on 2026/10/16.
//

#include "arm.h"
#include "main.h"

constexpr float kPi = 3.14159265358979323846f;
constexpr float kRadToDeg = 180.f / kPi;
constexpr float kDegToRad = kPi / 180.f;

Arm::Arm(Motor& joint0, Motor& joint1, const Limits& limits,
         const Feedforward& feedforward)
    : joint0_(joint0), joint1_(joint1), limits_(limits),
      feedforward_(feedforward) {}

bool Arm::feedbackReady() {
  return joint0_.connect_.check() && joint1_.connect_.check();
}

bool Arm::withinLimits(const float q[2]) const {
  return q[0] >= limits_.q_min[0] && q[0] <= limits_.q_max[0] &&
         q[1] >= limits_.q_min[1] && q[1] <= limits_.q_max[1];
}

bool Arm::calibrate(float q0_rad, float q1_rad) {
  if (mode_ != Mode::Disabled) return false;
  if (!feedbackReady()) return false;

  joint0_.resetFeedbackAngle(q0_rad * kRadToDeg);
  joint1_.resetFeedbackAngle(q1_rad * kRadToDeg);
  joint0_.targetAngle() = joint0_.realAngle();
  joint1_.targetAngle() = joint1_.realAngle();
  joint0_.targetSpeed() = 0.f;
  joint1_.targetSpeed() = 0.f;

  state_.q[0] = q0_rad;
  state_.q[1] = q1_rad;
  state_.dq[0] = state_.dq[1] = 0.f;

  reference_.q[0] = q0_rad;
  reference_.q[1] = q1_rad;
  reference_.dq[0] = reference_.dq[1] = 0.f;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;

  calibrated_ = true;
  return true;
}

bool Arm::enable() {
  if (mode_ == Mode::Joint) return true;
  if (!calibrated_ || !feedbackReady()) return false;

  updateFeedback();
  reference_.q[0] = state_.q[0];
  reference_.q[1] = state_.q[1];
  reference_.dq[0] = reference_.dq[1] = 0.f;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;

  joint0_.targetAngle() = joint0_.realAngle();
  joint1_.targetAngle() = joint1_.realAngle();
  joint0_.targetSpeed() = 0.f;
  joint1_.targetSpeed() = 0.f;

  joint0_.mode_ = Motor::WORKING;
  joint1_.mode_ = Motor::WORKING;

  mode_ = Mode::Joint;
  trajectory_type_ = TrajectoryType::None;
  trajectory_finished_ = false;
  return true;
}

void Arm::disable() {
  trajAbort();
  joint0_.targetSpeed() = 0.f;
  joint1_.targetSpeed() = 0.f;
  joint0_.mode_ = Motor::POWEROFF;
  joint1_.mode_ = Motor::POWEROFF;
  mode_ = Mode::Disabled;
}

bool Arm::setReference(const Reference& reference) {
  if (trajectory_type_ != TrajectoryType::None) return false;
  if (!withinLimits(reference.q)) return false;
  reference_ = reference;
  return true;
}

bool Arm::applyReference(const Reference& reference) {
  if (!withinLimits(reference.q)) return false;

  const float ff0 = reference.ddq[0] * feedforward_.k_acc[0];
  const float ff1 = reference.ddq[1] * feedforward_.k_acc[1];

  joint0_.setAngleSpeed(reference.q[0] * kRadToDeg,
                        reference.dq[0] * kRadToDeg, ff0);
  joint1_.setAngleSpeed(reference.q[1] * kRadToDeg,
                        reference.dq[1] * kRadToDeg, ff1);
  return true;
}

bool Arm::addJointReference(float speed0, float speed1, float dt_s) {
  if (mode_ != Mode::Joint) return false;
  if (trajectory_type_ != TrajectoryType::None) return false;

  float q0 = reference_.q[0] + speed0 * dt_s;
  float q1 = reference_.q[1] + speed1 * dt_s;
  float dq0 = speed0;
  float dq1 = speed1;

  if (q0 < limits_.q_min[0]) {
    q0 = limits_.q_min[0];
    dq0 = 0.f;
  } else if (q0 > limits_.q_max[0]) {
    q0 = limits_.q_max[0];
    dq0 = 0.f;
  }
  if (q1 < limits_.q_min[1]) {
    q1 = limits_.q_min[1];
    dq1 = 0.f;
  } else if (q1 > limits_.q_max[1]) {
    q1 = limits_.q_max[1];
    dq1 = 0.f;
  }

  reference_.q[0] = q0;
  reference_.q[1] = q1;
  reference_.dq[0] = dq0;
  reference_.dq[1] = dq1;
  reference_.ddq[0] = reference_.ddq[1] = 0.f;
  return true;
}

bool Arm::trajectoryReady() {
  return trajectory_type_ == TrajectoryType::None;
}

bool Arm::trajStart(float x_m, float y_m, float duration_s) {
  if (mode_ != Mode::Joint || !trajectoryReady()) return false;
  if (duration_s <= 0.f) return false;

  const CartesianReference start = ArmKinematics::forward(reference_.q);
  CartesianReference end;
  end.x = x_m;
  end.y = y_m;

  if (!ArmKinematics::lineReachable(start, end)) return false;

  constexpr int kSamples = 100;
  float prev[2] = {reference_.q[0], reference_.q[1]};
  for (int i = 0; i <= kSamples; ++i) {
    const float t = static_cast<float>(i) / kSamples;
    CartesianReference p;
    p.x = start.x + (end.x - start.x) * t;
    p.y = start.y + (end.y - start.y) * t;
    float q[2];
    if (!ArmKinematics::inverse(p, prev, trajectory_elbow_, q)) return false;
    if (!withinLimits(q)) return false;
    prev[0] = q[0];
    prev[1] = q[1];
  }
  if (!trajectory_.start(start, end, duration_s)) return false;
  trajectory_type_ = TrajectoryType::Cartesian;
  trajectory_start_tick_ = HAL_GetTick();
  trajectory_finished_ = false;
  return true;
}

bool Arm::trajStartJoint(float q0_rad, float q1_rad, float duration_s) {
    if (mode_ != Mode::Joint || !trajectoryReady()) return false;
    if (duration_s <= 0.f) return false;

    const float end[2] = {q0_rad, q1_rad};
    if (!withinLimits(end)) return false;

    joint_trajectory_start_[0] = reference_.q[0];
    joint_trajectory_start_[1] = reference_.q[1];
    joint_trajectory_end_[0] = q0_rad;
    joint_trajectory_end_[1] = q1_rad;
    joint_trajectory_duration_ = duration_s;

    trajectory_type_ = TrajectoryType::Joint;
    trajectory_start_tick_ = HAL_GetTick();
    trajectory_finished_ = false;
    return true;
}

void Arm::trajAbort() {
    trajectory_.abort();
    trajectory_type_ = TrajectoryType::None;
    trajectory_finished_ = false;
    reference_.dq[0] = reference_.dq[1] = 0.f;
    reference_.ddq[0] = reference_.ddq[1] = 0.f;
}

void Arm::trajectoryPlanner() {
    const float elapsed_s =static_cast<float>(HAL_GetTick() - trajectory_start_tick_) * 0.001f;

    if (trajectory_type_ == TrajectoryType::Cartesian) {
      CartesianReference p = trajectory_.sample(elapsed_s);

      float q[2];
      if (!ArmKinematics::inverse(p, reference_.q, trajectory_elbow_, q)) {
        trajAbort();
        return;
      }

      float dq[2] = {0.f, 0.f};
      float ddq[2] = {0.f, 0.f};
      if (!ArmKinematics::derivatives(p, q, dq, ddq)) {
        dq[0] = dq[1] = 0.f;
        ddq[0] = ddq[1] = 0.f;
      }

      reference_.q[0] = q[0];
      reference_.q[1] = q[1];
      reference_.dq[0] = dq[0];
      reference_.dq[1] = dq[1];
      reference_.ddq[0] = ddq[0];
      reference_.ddq[1] = ddq[1];

      if (!trajectory_.running()) {
        trajectory_type_ = TrajectoryType::None;
        trajectory_finished_ = true;
        reference_.dq[0] = reference_.dq[1] = 0.f;
        reference_.ddq[0] = reference_.ddq[1] = 0.f;
      }
    } else if (trajectory_type_ == TrajectoryType::Joint) {
      float pos[2], vel[2], acc[2];
      for (int i = 0; i < 2; ++i) {
        ArmTrajectory::interpolate(joint_trajectory_start_[i],
                                   joint_trajectory_end_[i], elapsed_s,
                                   joint_trajectory_duration_, pos[i], vel[i],
                                   acc[i]);
      }

      reference_.q[0] = pos[0];
      reference_.q[1] = pos[1];
      reference_.dq[0] = vel[0];
      reference_.dq[1] = vel[1];
      reference_.ddq[0] = acc[0];
      reference_.ddq[1] = acc[1];

      if (elapsed_s >= joint_trajectory_duration_) {
        reference_.q[0] = joint_trajectory_end_[0];
        reference_.q[1] = joint_trajectory_end_[1];
        reference_.dq[0] = reference_.dq[1] = 0.f;
        reference_.ddq[0] = reference_.ddq[1] = 0.f;
        trajectory_type_ = TrajectoryType::None;
        trajectory_finished_ = true;
      }
    }
}

void Arm::updateFeedback() {
  state_.q[0] = joint0_.realAngle() * kDegToRad;
  state_.q[1] = joint1_.realAngle() * kDegToRad;
  state_.dq[0] = joint0_.realSpeed() * kDegToRad;
  state_.dq[1] = joint1_.realSpeed() * kDegToRad;
}

void Arm::handle() {
  updateFeedback();

  if (mode_ == Mode::Disabled) {
    writeMotorCommands();
    return;
  }

  if (trajectory_type_ != TrajectoryType::None) {
    trajectoryPlanner();
  }

  if (!withinLimits(state_.q)) {
    disable();
    writeMotorCommands();
    return;
  }

  writeMotorCommands();
}

void Arm::writeMotorCommands() {
  if (mode_ == Mode::Disabled) {
    joint0_.targetSpeed() = 0.f;
    joint1_.targetSpeed() = 0.f;
    return;
  }
  applyReference(reference_);
}
