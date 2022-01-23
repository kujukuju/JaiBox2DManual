#include "dynamics/joints/friction_joint.h"

b2FrictionJointDef* FrictionJointDef_new() {
    return new b2FrictionJointDef();
}

b2FrictionJointDef FrictionJointDef_create() {
    return b2FrictionJointDef();
}

void FrictionJointDef_initialize(b2FrictionJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchor) {
    self->Initialize(bodyA, bodyB, *anchor);
}

b2Vec2 FrictionJoint_get_anchor_a(b2FrictionJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 FrictionJoint_get_anchor_b(b2FrictionJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 FrictionJoint_get_reaction_force(b2FrictionJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float FrictionJoint_get_reaction_torque(b2FrictionJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Vec2 FrictionJoint_get_local_anchor_a(b2FrictionJoint* self) {
    return self->GetLocalAnchorA();
}

b2Vec2 FrictionJoint_get_local_anchor_b(b2FrictionJoint* self) {
    return self->GetLocalAnchorB();
}

void FrictionJoint_set_max_force(b2FrictionJoint* self, float force) {
    self->SetMaxForce(force);
}

float FrictionJoint_get_max_force(b2FrictionJoint* self) {
    return self->GetMaxForce();
}

void FrictionJoint_set_max_torque(b2FrictionJoint* self, float torque) {
    self->SetMaxTorque(torque);
}

float FrictionJoint_get_max_torque(b2FrictionJoint* self) {
    return self->GetMaxTorque();
}

void FrictionJoint_dump(b2FrictionJoint* self) {
    self->Dump();
}
