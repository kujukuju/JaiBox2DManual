#include "dynamics/joints/prismatic_joint.h"

b2Joint* World_create_prismatic_joint(
    b2World* self,
    b2Body* body_a,
    b2Body* body_b,
    bool collide_connected,
    b2Vec2 local_anchor_a,
    b2Vec2 local_anchor_b,
    b2Vec2 local_axis_a,
    float reference_angle,
    bool enable_limit,
    float lower_translation,
    float upper_translation,
    bool enable_motor,
    float max_motor_force,
    float motor_speed
) {
    b2PrismaticJointDef def;
    def.bodyA = body_a;
    def.bodyB = body_b;
    def.collideConnected = collide_connected;
    def.localAnchorA = local_anchor_a;
    def.localAnchorB = local_anchor_b;
    def.localAxisA = local_axis_a;
    def.referenceAngle = reference_angle;
    def.enableLimit = enable_limit;
    def.lowerTranslation = lower_translation;
    def.upperTranslation = upper_translation;
    def.enableMotor = enable_motor;
    def.maxMotorForce = max_motor_force;
    def.motorSpeed = motor_speed;

    return self->CreateJoint(&def);
}

void PrismaticJointDef_initialize(b2PrismaticJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor,
                                 const b2Vec2* axis) {
    self->Initialize(body_a, body_b, *anchor, *axis);
}

b2Joint* PrismaticJoint_as_joint(b2PrismaticJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2PrismaticJoint* Joint_as_prismatic_joint(b2Joint* self) {
    return static_cast<b2PrismaticJoint*>(self);
}

const b2Vec2* PrismaticJoint_get_local_anchor_a(const b2PrismaticJoint* self) {
    return &self->GetLocalAnchorA();
}
const b2Vec2* PrismaticJoint_get_local_anchor_b(const b2PrismaticJoint* self) {
    return &self->GetLocalAnchorB();
}
const b2Vec2* PrismaticJoint_get_local_axis_a(const b2PrismaticJoint* self) {
    return &self->GetLocalAxisA();
}
float PrismaticJoint_get_reference_angle(const b2PrismaticJoint* self) {
    return self->GetReferenceAngle();
}
float PrismaticJoint_get_joint_translation(const b2PrismaticJoint* self) {
    return self->GetJointTranslation();
}
float PrismaticJoint_get_joint_speed(const b2PrismaticJoint* self) {
    return self->GetJointSpeed();
}
bool PrismaticJoint_is_limit_enabled(const b2PrismaticJoint* self) {
    return self->IsLimitEnabled();
}
void PrismaticJoint_enable_limit(b2PrismaticJoint* self, bool flag) {
    self->EnableLimit(flag);
}
float PrismaticJoint_get_lower_limit(const b2PrismaticJoint* self) {
    return self->GetLowerLimit();
}
float PrismaticJoint_get_upper_limit(const b2PrismaticJoint* self) {
    return self->GetUpperLimit();
}
void PrismaticJoint_set_limits(b2PrismaticJoint* self, float lower, float upper) {
    self->SetLimits(lower, upper);
}
bool PrismaticJoint_is_motor_enabled(const b2PrismaticJoint* self) {
    return self->IsMotorEnabled();
}
void PrismaticJoint_enable_motor(b2PrismaticJoint* self, bool flag) {
    self->EnableMotor(flag);
}
void PrismaticJoint_set_motor_speed(b2PrismaticJoint* self, float speed) {
    self->SetMotorSpeed(speed);
}
float PrismaticJoint_get_motor_speed(const b2PrismaticJoint* self) {
    return self->GetMotorSpeed();
}
void PrismaticJoint_set_max_motor_force(b2PrismaticJoint* self, float force) {
    self->SetMaxMotorForce(force);
}
float PrismaticJoint_get_max_motor_force(const b2PrismaticJoint* self) {
    return self->GetMaxMotorForce();
}
float PrismaticJoint_get_motor_force(const b2PrismaticJoint* self, float inv_dt) {
    return self->GetMotorForce(inv_dt);
}
