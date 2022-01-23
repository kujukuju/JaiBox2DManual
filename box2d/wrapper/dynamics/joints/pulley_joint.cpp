#include "dynamics/joints/pulley_joint.h"

b2PulleyJointDef* PulleyJointDef_new() {
    return new b2PulleyJointDef();
}

b2PulleyJointDef PulleyJointDef_create() {
    return b2PulleyJointDef();
}

void PulleyJointDef_initialize(b2PulleyJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* groundAnchorA, const b2Vec2* groundAnchorB, const b2Vec2* anchorA, const b2Vec2* anchorB, float ratio) {
    self->Initialize(bodyA, bodyB, *groundAnchorA, *groundAnchorB, *anchorA, *anchorB, ratio);
}

b2Vec2 PulleyJoint_get_anchor_a(b2PulleyJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 PulleyJoint_get_anchor_b(b2PulleyJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 PulleyJoint_get_reaction_force(b2PulleyJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float PulleyJoint_get_reaction_torque(b2PulleyJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Vec2 PulleyJoint_get_ground_anchor_a(b2PulleyJoint* self) {
    return self->GetGroundAnchorA();
}

b2Vec2 PulleyJoint_get_ground_anchor_b(b2PulleyJoint* self) {
    return self->GetGroundAnchorB();
}

float PulleyJoint_get_length_a(b2PulleyJoint* self) {
    return self->GetLengthA();
}

float PulleyJoint_get_length_b(b2PulleyJoint* self) {
    return self->GetLengthB();
}

float PulleyJoint_get_ratio(b2PulleyJoint* self) {
    return self->GetRatio();
}

float PulleyJoint_get_current_length_a(b2PulleyJoint* self) {
    return self->GetCurrentLengthA();
}

float PulleyJoint_get_current_length_b(b2PulleyJoint* self) {
    return self->GetCurrentLengthB();
}

void PulleyJoint_dump(b2PulleyJoint* self) {
    self->Dump();
}

void PulleyJoint_shift_origin(b2PulleyJoint* self, const b2Vec2* newOrigin) {
    self->ShiftOrigin(*newOrigin);
}
