#include "dynamics/joints/pulley_joint.h"

b2Joint* World_create_pulley_joint(
    b2World* self,
    b2Body* body_a,
    b2Body* body_b,
    bool collide_connected,
    b2Vec2 ground_anchor_a,
    b2Vec2 ground_anchor_b,
    b2Vec2 local_anchor_a,
    b2Vec2 local_anchor_b,
    float length_a,
    float length_b,
    float ratio
) {
    b2PulleyJointDef def;
    def.bodyA = body_a;
    def.bodyB = body_b;
    def.collideConnected = collide_connected;
    def.groundAnchorA = ground_anchor_a;
    def.groundAnchorB = ground_anchor_b;
    def.localAnchorA = local_anchor_a;
    def.localAnchorB = local_anchor_b;
    def.lengthA = length_a;
    def.lengthB = length_b;
    def.ratio = ratio;

    return self->CreateJoint(&def);
}

void PulleyJointDef_initialize(b2PulleyJointDef* self,
                               b2Body* body_a, b2Body* body_b,
                               const b2Vec2* ground_anchor_a,
                               const b2Vec2* ground_anchor_b,
                               const b2Vec2* anchor_a,
                               const b2Vec2* anchor_b,
                               float ratio) {
    self->Initialize(body_a, body_b,
                     *ground_anchor_a, *ground_anchor_b,
                     *anchor_a, *anchor_b,
                     ratio);
}

b2Joint* PulleyJoint_as_joint(b2PulleyJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2PulleyJoint* Joint_as_pulley_joint(b2Joint* self) {
    return static_cast<b2PulleyJoint*>(self);
}

b2Vec2 PulleyJoint_get_ground_anchor_a(const b2PulleyJoint* self) {
    return self->GetGroundAnchorA();
}
b2Vec2 PulleyJoint_get_ground_anchor_b(const b2PulleyJoint* self) {
    return self->GetGroundAnchorB();
}
float PulleyJoint_get_length_a(const b2PulleyJoint* self) {
    return self->GetLengthA();
}
float PulleyJoint_get_length_b(const b2PulleyJoint* self) {
    return self->GetLengthB();
}
float PulleyJoint_get_ratio(const b2PulleyJoint* self) {
    return self->GetRatio();
}
float PulleyJoint_get_current_length_a(const b2PulleyJoint* self) {
    return self->GetCurrentLengthA();
}
float PulleyJoint_get_current_length_b(const b2PulleyJoint* self) {
    return self->GetCurrentLengthB();
}
