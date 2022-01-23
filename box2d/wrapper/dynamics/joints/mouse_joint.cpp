#include "dynamics/joints/mouse_joint.h"

b2MouseJointDef* MouseJointDef_new() {
    return new b2MouseJointDef();
}

b2MouseJointDef MouseJointDef_create() {
    return b2MouseJointDef();
}

b2Vec2 MouseJoint_get_anchor_a(b2MouseJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 MouseJoint_get_anchor_b(b2MouseJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 MouseJoint_get_reaction_force(b2MouseJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float MouseJoint_get_reaction_torque(b2MouseJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

void MouseJoint_set_target(b2MouseJoint* self, const b2Vec2* target) {
    self->SetTarget(*target);
}

b2Vec2 MouseJoint_get_target(b2MouseJoint* self) {
    return self->GetTarget();
}

void MouseJoint_set_max_force(b2MouseJoint* self, float force) {
    self->SetMaxForce(force);
}

float MouseJoint_get_max_force(b2MouseJoint* self) {
    return self->GetMaxForce();
}

void MouseJoint_set_stiffness(b2MouseJoint* self, float stiffness) {
    self->SetStiffness(stiffness);
}

float MouseJoint_get_stiffness(b2MouseJoint* self) {
    return self->GetStiffness();
}

void MouseJoint_set_damping(b2MouseJoint* self, float damping) {
    self->SetDamping(damping);
}

float MouseJoint_get_damping(b2MouseJoint* self) {
    return self->GetDamping();
}

void MouseJoint_dump(b2MouseJoint* self) {
    self->Dump();
}

void MouseJoint_shift_origin(b2MouseJoint* self, const b2Vec2* newOrigin) {
    self->ShiftOrigin(*newOrigin);
}

