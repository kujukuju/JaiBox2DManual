#include "dynamics/joints/distance_joint.h"

b2DistanceJointDef* DistanceJointDef_new() {
    return new b2DistanceJointDef();
}

b2DistanceJointDef DistanceJointDef_create() {
    return b2DistanceJointDef();
}

void DistanceJointDef_initialize(b2DistanceJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchorA, const b2Vec2* anchorB) {
    self->Initialize(bodyA, bodyB, *anchorA, *anchorB);
}

b2Vec2 DistanceJoint_get_anchor_a(b2DistanceJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 DistanceJoint_get_anchor_b(b2DistanceJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 DistanceJoint_get_reaction_force(b2DistanceJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float DistanceJoint_get_reaction_torque(b2DistanceJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Vec2 DistanceJoint_get_local_anchor_a(b2DistanceJoint* self) {
    return self->GetLocalAnchorA();
}

b2Vec2 DistanceJoint_get_local_anchor_b(b2DistanceJoint* self) {
    return self->GetLocalAnchorB();
}

float DistanceJoint_get_length(b2DistanceJoint* self) {
    return self->GetLength();
}

float DistanceJoint_set_length(b2DistanceJoint* self, float length) {
    return self->SetLength(length);
}

float DistanceJoint_get_min_length(b2DistanceJoint* self) {
    return self->GetMinLength();
}

float DistanceJoint_set_min_length(b2DistanceJoint* self, float minLength) {
    return self->SetMinLength(minLength);
}

float DistanceJoint_get_max_length(b2DistanceJoint* self) {
    return self->GetMaxLength();
}

float DistanceJoint_set_max_length(b2DistanceJoint* self, float maxLength) {
    return self->SetMaxLength(maxLength);
}

float DistanceJoint_get_current_length(b2DistanceJoint* self) {
    return self->GetCurrentLength();
}

void DistanceJoint_set_stiffness(b2DistanceJoint* self, float stiffness) {
    self->SetStiffness(stiffness);
}

float DistanceJoint_get_stiffness(b2DistanceJoint* self) {
    return self->GetStiffness();
}

void DistanceJoint_set_damping(b2DistanceJoint* self, float damping) {
    self->SetDamping(damping);
}

float DistanceJoint_get_damping(b2DistanceJoint* self) {
    return self->GetDamping();
}

void DistanceJoint_dump(b2DistanceJoint* self) {
    self->Dump();
}

void DistanceJoint_draw(b2DistanceJoint* self, b2Draw* draw) {
    self->Draw(draw);
}
