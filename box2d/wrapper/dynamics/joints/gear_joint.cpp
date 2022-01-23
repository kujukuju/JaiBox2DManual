#include "dynamics/joints/gear_joint.h"

b2GearJointDef* GearJointDef_new() {
    return new b2GearJointDef();
}

b2GearJointDef GearJointDef_create() {
    return b2GearJointDef();
}

b2Vec2 GearJoint_get_anchor_a(b2GearJoint* self) {
    return self->GetAnchorA();
}

b2Vec2 GearJoint_get_anchor_b(b2GearJoint* self) {
    return self->GetAnchorB();
}

b2Vec2 GearJoint_get_reaction_force(b2GearJoint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float GearJoint_get_reaction_torque(b2GearJoint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Joint* GearJoint_get_joint1(b2GearJoint* self) {
    return self->GetJoint1();
}

b2Joint* GearJoint_get_joint2(b2GearJoint* self) {
    return self->GetJoint2();
}

void GearJoint_set_ratio(b2GearJoint* self, float ratio) {
    self->SetRatio(ratio);
}

float GearJoint_get_ratio(b2GearJoint* self) {
    return self->GetRatio();
}

void GearJoint_dump(b2GearJoint* self) {
    self->Dump();
}

