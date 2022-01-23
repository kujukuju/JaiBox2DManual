#include "dynamics/joints/revolute_joint.h"

b2RevoluteJointDef* RevoluteJointDef_new() {
    return new b2RevoluteJointDef();
}

b2RevoluteJointDef RevoluteJointDef_create() {
    return b2RevoluteJointDef();
}

void RevoluteJointDef_initialize(b2RevoluteJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchor) {
    self->Initialize(bodyA, bodyB, *anchor);
}

b2Vec2 RevoluteJoint_get_anchor_a(b2RevoluteJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 RevoluteJoint_get_anchor_b(b2RevoluteJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 RevoluteJoint_get_local_anchor_a(b2RevoluteJoint* self) {
    return self->GetLocalAnchorA();
}

b2Vec2 RevoluteJoint_get_local_anchor_b(b2RevoluteJoint* self) {
    return self->GetLocalAnchorB();
}

float RevoluteJoint_get_reference_angle(b2RevoluteJoint* self) {
    return self->GetReferenceAngle();
}

float RevoluteJoint_get_joint_angle(b2RevoluteJoint* self) {
    return self->GetJointAngle();
}

float RevoluteJoint_get_joint_speed(b2RevoluteJoint* self) {
    return self->GetJointSpeed();
}

bool RevoluteJoint_is_limit_enabled(b2RevoluteJoint* self) {
    return self->IsLimitEnabled();
}

void RevoluteJoint_enable_limit(b2RevoluteJoint* self, bool flag) {
    self->EnableLimit(flag);
}

float RevoluteJoint_get_lower_limit(b2RevoluteJoint* self) {
    return self->GetLowerLimit();
}

float RevoluteJoint_get_upper_limit(b2RevoluteJoint* self) {
    return self->GetUpperLimit();
}

void RevoluteJoint_set_limits(b2RevoluteJoint* self, float lower, float upper) {
    self->SetLimits(lower, upper);
}

bool RevoluteJoint_is_motor_enabled(b2RevoluteJoint* self) {
    return self->IsMotorEnabled();
}

void RevoluteJoint_enable_motor(b2RevoluteJoint* self, bool flag) {
    self->EnableMotor(flag);
}

void RevoluteJoint_set_motor_speed(b2RevoluteJoint* self, float speed) {
    self->SetMotorSpeed(speed);
}

float RevoluteJoint_get_motor_speed(b2RevoluteJoint* self) {
    return self->GetMotorSpeed();
}

void RevoluteJoint_set_max_motor_torque(b2RevoluteJoint* self, float torque) {
    self->SetMaxMotorTorque(torque);
}

float RevoluteJoint_get_max_motor_torque(b2RevoluteJoint* self) {
    return self->GetMaxMotorTorque();
}

b2Vec2 RevoluteJoint_get_reaction_force(b2RevoluteJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float RevoluteJoint_get_reaction_torque(b2RevoluteJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

float RevoluteJoint_get_motor_torque(b2RevoluteJoint* self, float inv_dt) {
    return self->GetMotorTorque(inv_dt);
}

void RevoluteJoint_dump(b2RevoluteJoint* self) {
    self->Dump();
}

void RevoluteJoint_draw(b2RevoluteJoint* self, b2Draw* draw) {
    self->Draw(draw);
}
