#include "dynamics/joints/wheel_joint.h"

b2WheelJointDef* WheelJointDef_new() {
    return new b2WheelJointDef();
}

b2WheelJointDef WheelJointDef_create() {
    return b2WheelJointDef();
}

void WheelJointDef_initialize(b2WheelJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchor, const b2Vec2* axis) {
    self->Initialize(bodyA, bodyB, *anchor, *axis);
}

b2Vec2 WheelJoint_get_anchor_a(b2WheelJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 WheelJoint_get_anchor_b(b2WheelJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 WheelJoint_get_reaction_force(b2WheelJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float WheelJoint_get_reaction_torque(b2WheelJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Vec2 WheelJoint_get_local_anchor_a(b2WheelJoint* self) {
    return self->GetLocalAnchorA();
}

b2Vec2 WheelJoint_get_local_anchor_b(b2WheelJoint* self) {
    return self->GetLocalAnchorB();
}

b2Vec2 WheelJoint_get_local_axis_a(b2WheelJoint* self) {
    return self->GetLocalAxisA();
}

float WheelJoint_get_joint_translation(b2WheelJoint* self) {
    return self->GetJointTranslation();
}

float WheelJoint_get_joint_linear_speed(b2WheelJoint* self) {
    return self->GetJointLinearSpeed();
}

float WheelJoint_get_joint_angle(b2WheelJoint* self) {
    return self->GetJointAngle();
}

float WheelJoint_get_joint_angular_speed(b2WheelJoint* self) {
    return self->GetJointAngularSpeed();
}

bool WheelJoint_is_limit_enabled(b2WheelJoint* self) {
    return self->IsLimitEnabled();
}

void WheelJoint_enable_limit(b2WheelJoint* self, bool flag) {
    self->EnableLimit(flag);
}

float WheelJoint_get_lower_limit(b2WheelJoint* self) {
    return self->GetLowerLimit();
}

float WheelJoint_get_upper_limit(b2WheelJoint* self) {
    return self->GetUpperLimit();
}

void WheelJoint_set_limits(b2WheelJoint* self, float lower, float upper) {
    self->SetLimits(lower, upper);
}

bool WheelJoint_is_motor_enabled(b2WheelJoint* self) {
    return self->IsMotorEnabled();
}

void WheelJoint_enable_motor(b2WheelJoint* self, bool flag) {
    self->EnableMotor(flag);
}

void WheelJoint_set_motor_speed(b2WheelJoint* self, float speed) {
    self->SetMotorSpeed(speed);
}

float WheelJoint_get_motor_speed(b2WheelJoint* self) {
    return self->GetMotorSpeed();
}

void WheelJoint_set_max_motor_torque(b2WheelJoint* self, float torque) {
    self->SetMaxMotorTorque(torque);
}

float WheelJoint_get_max_motor_torque(b2WheelJoint* self) {
    return self->GetMaxMotorTorque();
}

float WheelJoint_get_motor_torque(b2WheelJoint* self, float inv_dt) {
    return self->GetMotorTorque(inv_dt);
}

void WheelJoint_set_stiffness(b2WheelJoint* self, float stiffness) {
    self->SetStiffness(stiffness);
}

float WheelJoint_get_stiffness(b2WheelJoint* self) {
    return self->GetStiffness();
}

void WheelJoint_set_damping(b2WheelJoint* self, float damping) {
    self->SetDamping(damping);
}

float WheelJoint_get_damping(b2WheelJoint* self) {
    return self->GetDamping();
}

void WheelJoint_dump(b2WheelJoint* self) {
    self->Dump();
}

void WheelJoint_draw(b2WheelJoint* self, b2Draw* draw) {
    self->Draw(draw);
}
