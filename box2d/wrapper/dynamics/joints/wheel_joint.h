EXPORT b2Joint* World_create_wheel_joint(
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
);
EXPORT void WheelJointDef_initialize(b2WheelJointDef* self,
                              b2Body* body_a, b2Body* body_b,
                              const b2Vec2* anchor,
                              const b2Vec2* axis);
EXPORT b2Joint* WheelJoint_as_joint(b2WheelJoint* self);
EXPORT b2WheelJoint* Joint_as_wheel_joint(b2Joint* self);
EXPORT const b2Vec2* WheelJoint_get_local_anchor_a(const b2WheelJoint* self);
EXPORT const b2Vec2* WheelJoint_get_local_anchor_b(const b2WheelJoint* self);
EXPORT const b2Vec2* WheelJoint_get_local_axis_a(const b2WheelJoint* self);
EXPORT float WheelJoint_get_joint_translation(const b2WheelJoint* self);
EXPORT float WheelJoint_get_joint_speed(const b2WheelJoint* self);
EXPORT bool WheelJoint_is_motor_enabled(const b2WheelJoint* self);
EXPORT void WheelJoint_enable_motor(b2WheelJoint* self, bool flag);
EXPORT void WheelJoint_set_motor_speed(b2WheelJoint* self, float speed);
EXPORT float WheelJoint_get_motor_speed(const b2WheelJoint* self);
EXPORT void WheelJoint_set_max_motor_torque(b2WheelJoint* self, float torque);
EXPORT float WheelJoint_get_max_motor_torque(const b2WheelJoint* self);
EXPORT float WheelJoint_get_motor_torque(const b2WheelJoint* self, float inv_dt);
EXPORT void WheelJoint_set_spring_frequency(b2WheelJoint* self, float frequency);
EXPORT float WheelJoint_get_spring_frequency(const b2WheelJoint* self);
EXPORT void WheelJoint_set_spring_damping_ratio(b2WheelJoint* self, float ratio);
EXPORT float WheelJoint_get_spring_damping_ratio(const b2WheelJoint* self);