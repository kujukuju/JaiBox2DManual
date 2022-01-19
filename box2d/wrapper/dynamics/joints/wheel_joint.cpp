#include "dynamics/joints/wheel_joint.h"

b2Joint* World_create_wheel_joint(
    b2World* world,
    b2Body* body_a,
    b2Body* body_b,
    bool collide_connected,
    b2Vec2 local_anchor_a,
    b2Vec2 local_anchor_b,
    b2Vec2 local_axis_a,
    bool enable_motor,
    float max_motor_torque,
    float motor_speed,
    float frequency,
    float damping_ratio
) {
    b2WheelJointDef def;
    def.bodyA = body_a;
    def.bodyB = body_b;
    def.collideConnected = collide_connected;
    def.localAnchorA = local_anchor_a;
    def.localAnchorB = local_anchor_b;
    def.localAxisA = local_axis_a;
    def.enableMotor = enable_motor;
    def.maxMotorTorque = max_motor_torque;
    def.motorSpeed = motor_speed;
    def.frequencyHz = frequency;
    def.dampingRatio = damping_ratio;

    return world->CreateJoint(&def);
}

void WheelJointDef_initialize(b2WheelJointDef* self,
                              b2Body* body_a, b2Body* body_b,
                              const b2Vec2* anchor,
                              const b2Vec2* axis) {
    self->Initialize(body_a, body_b, *anchor, *axis);
}

b2Joint* WheelJoint_as_joint(b2WheelJoint* self) {
    return static_cast<b2Joint*>(self);
}
b2WheelJoint* Joint_as_wheel_joint(b2Joint* self) {
    return static_cast<b2WheelJoint*>(self);
}

const b2Vec2* WheelJoint_get_local_anchor_a(const b2WheelJoint* self) {
    return &self->GetLocalAnchorA();
}
const b2Vec2* WheelJoint_get_local_anchor_b(const b2WheelJoint* self) {
    return &self->GetLocalAnchorB();
}
const b2Vec2* WheelJoint_get_local_axis_a(const b2WheelJoint* self) {
    return &self->GetLocalAxisA();
}
float WheelJoint_get_joint_translation(const b2WheelJoint* self) {
    return self->GetJointTranslation();
}
float WheelJoint_get_joint_speed(const b2WheelJoint* self) {
    return self->GetJointSpeed();
}
bool WheelJoint_is_motor_enabled(const b2WheelJoint* self) {
    return self->IsMotorEnabled();
}
void WheelJoint_enable_motor(b2WheelJoint* self, bool flag) {
    self->EnableMotor(flag);
}
void WheelJoint_set_motor_speed(b2WheelJoint* self, float speed) {
    self->SetMotorSpeed(speed);
}
float WheelJoint_get_motor_speed(const b2WheelJoint* self) {
    return self->GetMotorSpeed();
}
void WheelJoint_set_max_motor_torque(b2WheelJoint* self, float torque) {
    self->SetMaxMotorTorque(torque);
}
float WheelJoint_get_max_motor_torque(const b2WheelJoint* self) {
    return self->GetMaxMotorTorque();
}
float WheelJoint_get_motor_torque(const b2WheelJoint* self, float inv_dt) {
    return self->GetMotorTorque(inv_dt);
}
void WheelJoint_set_spring_frequency(b2WheelJoint* self, float frequency) {
    self->SetSpringFrequencyHz(frequency);
}
float WheelJoint_get_spring_frequency(const b2WheelJoint* self) {
    return self->GetSpringFrequencyHz();
}
void WheelJoint_set_spring_damping_ratio(b2WheelJoint* self, float ratio) {
    self->SetSpringDampingRatio(ratio);
}
float WheelJoint_get_spring_damping_ratio(const b2WheelJoint* self) {
    return self->GetSpringDampingRatio();
}
