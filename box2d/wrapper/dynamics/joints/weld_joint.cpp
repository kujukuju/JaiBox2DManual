#include "dynamics/joints/weld_joint.h"

b2WeldJointDef* WeldJointDef_new() {
    return new b2WeldJointDef();
}

b2WeldJointDef WeldJointDef_create() {
    return b2WeldJointDef();
}

void WeldJointDef_initialize(b2WeldJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchor) {
    self->Initialize(bodyA, bodyB, *anchor);
}

b2Vec2 WeldJoint_get_anchor_a(b2WeldJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 WeldJoint_get_anchor_b(b2WeldJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 WeldJoint_get_reaction_force(b2WeldJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float WeldJoint_get_reaction_torque(b2WeldJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Vec2 WeldJoint_get_local_anchor_a(b2WeldJoint* self) {
    return self->GetLocalAnchorA();
}

b2Vec2 WeldJoint_get_local_anchor_b(b2WeldJoint* self) {
    return self->GetLocalAnchorB();
}

float WeldJoint_get_reference_angle(b2WeldJoint* self) {
    return self->GetReferenceAngle();
}

void WeldJoint_set_stiffness(b2WeldJoint* self, float stiffness) {
    self->SetStiffness(stiffness);
}

float WeldJoint_get_stiffness(b2WeldJoint* self) {
    return self->GetStiffness();
}

void WeldJoint_set_damping(b2WeldJoint* self, float damping) {
    self->SetDamping(damping);
}

float WeldJoint_get_damping(b2WeldJoint* self) {
    return self->GetDamping();
}

void WeldJoint_dump(b2WeldJoint* self) {
    self->Dump();
}
