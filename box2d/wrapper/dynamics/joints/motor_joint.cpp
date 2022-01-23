#include "dynamics/joints/motor_joint.h"

b2MotorJointDef* MotorJointDef_new() {
    return new b2MotorJointDef();
}

b2MotorJointDef MotorJointDef_create() {
    return b2MotorJointDef();
}

void MotorJointDef_initialize(b2MotorJointDef* self, b2Body* bodyA, b2Body* bodyB) {
    self->Initialize(bodyA, bodyB);
}

b2Vec2 MotorJoint_get_anchor_a(b2MotorJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 MotorJoint_get_anchor_b(b2MotorJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 MotorJoint_get_reaction_force(b2MotorJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float MotorJoint_get_reaction_torque(b2MotorJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

void MotorJoint_set_linear_offset(b2MotorJoint* self, const b2Vec2* linearOffset) {
    self->SetLinearOffset(*linearOffset);
}

b2Vec2 MotorJoint_get_linear_offset(b2MotorJoint* self) {
    return self->GetLinearOffset();
}

void MotorJoint_set_angular_offset(b2MotorJoint* self, float angularOffset) {
    self->SetAngularOffset(angularOffset);
}

float MotorJoint_get_angular_offset(b2MotorJoint* self) {
    return self->GetAngularOffset();
}

void MotorJoint_set_max_force(b2MotorJoint* self, float force) {
    self->SetMaxForce(force);
}

float MotorJoint_get_max_force(b2MotorJoint* self) {
    return self->GetMaxForce();
}

void MotorJoint_set_max_torque(b2MotorJoint* self, float torque) {
    self->SetMaxTorque(torque);
}

float MotorJoint_get_max_torque(b2MotorJoint* self) {
    return self->GetMaxTorque();
}

void MotorJoint_set_correction_factor(b2MotorJoint* self, float factor) {
    self->SetCorrectionFactor(factor);
}

float MotorJoint_get_correction_factor(b2MotorJoint* self) {
    return self->GetCorrectionFactor();
}

void MotorJoint_dump(b2MotorJoint* self) {
    self->Dump();
}
