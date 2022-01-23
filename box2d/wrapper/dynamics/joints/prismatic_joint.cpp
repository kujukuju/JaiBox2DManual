#include "dynamics/joints/prismatic_joint.h"

b2PrismaticJointDef* PrismaticJointDef_new() {
    return new b2PrismaticJointDef();
}

b2PrismaticJointDef PrismaticJointDef_create() {
    return b2PrismaticJointDef();
}

void PrismaticJointDef_initialize(b2PrismaticJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchor, const b2Vec2* axis) {
    self->Initialize(bodyA, bodyB, *anchor, *axis);
}

b2Vec2 PrismaticJoint_get_anchor_a(b2PrismaticJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 PrismaticJoint_get_anchor_b(b2PrismaticJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 PrismaticJoint_get_reaction_force(b2PrismaticJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float PrismaticJoint_get_reaction_torque(b2PrismaticJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Vec2 PrismaticJoint_get_local_anchor_a(b2PrismaticJoint* self) {
    return self->GetLocalAnchorA();
}

b2Vec2 PrismaticJoint_get_local_anchor_b(b2PrismaticJoint* self) {
    return self->GetLocalAnchorB();
}

b2Vec2 PrismaticJoint_get_local_axis_a(b2PrismaticJoint* self) {
    return self->GetLocalAxisA();
}

float PrismaticJoint_get_reference_angle(b2PrismaticJoint* self) {
    return self->GetReferenceAngle();
}

float PrismaticJoint_get_joint_translation(b2PrismaticJoint* self) {
    return self->GetJointTranslation();
}

float PrismaticJoint_get_joint_speed(b2PrismaticJoint* self) {
    return self->GetJointSpeed();
}

bool PrismaticJoint_is_limit_enabled(b2PrismaticJoint* self) {
    return self->IsLimitEnabled();
}

void PrismaticJoint_enable_limit(b2PrismaticJoint* self, bool flag) {
    self->EnableLimit(flag);
}

float PrismaticJoint_get_lower_limit(b2PrismaticJoint* self) {
    return self->GetLowerLimit();
}

float PrismaticJoint_get_upper_limit(b2PrismaticJoint* self) {
    return self->GetUpperLimit();
}

void PrismaticJoint_set_limits(b2PrismaticJoint* self, float lower, float upper) {
    self->SetLimits(lower, upper);
}

bool PrismaticJoint_is_motor_enabled(b2PrismaticJoint* self) {
    return self->IsMotorEnabled();
}

void PrismaticJoint_enable_motor(b2PrismaticJoint* self, bool flag) {
    self->EnableMotor(flag);
}

void PrismaticJoint_set_motor_speed(b2PrismaticJoint* self, float speed) {
    self->SetMotorSpeed(speed);
}

float PrismaticJoint_get_motor_speed(b2PrismaticJoint* self) {
    return self->GetMotorSpeed();
}

void PrismaticJoint_set_max_motor_force(b2PrismaticJoint* self, float force) {
    self->SetMaxMotorForce(force);
}

float PrismaticJoint_get_max_motor_force(b2PrismaticJoint* self) {
    return self->GetMaxMotorForce();
}

float PrismaticJoint_get_motor_force(b2PrismaticJoint* self, float inv_dt) {
    return self->GetMotorForce(inv_dt);
}

void PrismaticJoint_dump(b2PrismaticJoint* self) {
    self->Dump();
}

void PrismaticJoint_draw(b2PrismaticJoint* self, b2Draw* draw) {
    self->Draw(draw);
}
