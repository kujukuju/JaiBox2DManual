#include "dynamics/joints/revolute_joint.h"

b2Joint* World_create_revolute_joint(
    b2World* world,
    b2Body* body_a,
    b2Body* body_b,
    bool collide_connected,
    b2Vec2 local_anchor_a,
    b2Vec2 local_anchor_b,
    float reference_angle,
    bool enable_limit,
    float lower_angle,
    float upper_angle,
    bool enable_motor,
    float motor_speed,
    float max_motor_torque
) {
    b2RevoluteJointDef def;
    def.bodyA = body_a;
    def.bodyB = body_b;
    def.collideConnected = collide_connected;
    def.localAnchorA = local_anchor_a;
    def.localAnchorB = local_anchor_b;
    def.referenceAngle = reference_angle;
    def.enableLimit = enable_limit;
    def.lowerAngle = lower_angle;
    def.upperAngle = upper_angle;
    def.enableMotor = enable_motor;
    def.motorSpeed = motor_speed;
    def.maxMotorTorque = max_motor_torque;

    return world->CreateJoint(&def);
}


void RevoluteJointDef_initialize(b2RevoluteJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor) {
    self->Initialize(body_a, body_b, *anchor);
}

b2Joint* RevoluteJoint_as_joint(b2RevoluteJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2RevoluteJoint* Joint_as_revolute_joint(b2Joint* self) {
    return static_cast<b2RevoluteJoint*>(self);
}

const b2Vec2* RevoluteJoint_get_local_anchor_a(const b2RevoluteJoint* self) {
    return &self->GetLocalAnchorA();
}
const b2Vec2* RevoluteJoint_get_local_anchor_b(const b2RevoluteJoint* self) {
    return &self->GetLocalAnchorB();
}
float RevoluteJoint_get_reference_angle(const b2RevoluteJoint* self) {
    return self->GetReferenceAngle();
}
float RevoluteJoint_get_joint_angle(const b2RevoluteJoint* self) {
    return self->GetJointAngle();
}
float RevoluteJoint_get_joint_speed(const b2RevoluteJoint* self) {
    return self->GetJointSpeed();
}
bool RevoluteJoint_is_limit_enabled(const b2RevoluteJoint* self) {
    return self->IsLimitEnabled();
}
void RevoluteJoint_enable_limit(b2RevoluteJoint* self, bool flag) {
    self->EnableLimit(flag);
}
float RevoluteJoint_get_lower_limit(const b2RevoluteJoint* self) {
    return self->GetLowerLimit();
}
float RevoluteJoint_get_upper_limit(const b2RevoluteJoint* self) {
    return self->GetUpperLimit();
}
void RevoluteJoint_set_limits(b2RevoluteJoint* self, float lower, float upper) {
    self->SetLimits(lower, upper);
}
bool RevoluteJoint_is_motor_enabled(const b2RevoluteJoint* self) {
    return self->IsMotorEnabled();
}
void RevoluteJoint_enable_motor(b2RevoluteJoint* self, bool flag) {
    self->EnableMotor(flag);
}
void RevoluteJoint_set_motor_speed(b2RevoluteJoint* self, float speed) {
    self->SetMotorSpeed(speed);
}
float RevoluteJoint_get_motor_speed(const b2RevoluteJoint* self) {
    return self->GetMotorSpeed();
}
void RevoluteJoint_set_max_motor_torque(b2RevoluteJoint* self, float torque) {
    self->SetMaxMotorTorque(torque);
}
float RevoluteJoint_get_max_motor_torque(const b2RevoluteJoint* self) {
    return self->GetMaxMotorTorque();
}
float RevoluteJoint_get_motor_torque(const b2RevoluteJoint* self, float inv_dt) {
    return self->GetMotorTorque(inv_dt);
}
