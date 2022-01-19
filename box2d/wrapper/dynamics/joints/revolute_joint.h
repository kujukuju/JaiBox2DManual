EXPORT b2Joint* World_create_revolute_joint(
        b2World* world,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 local_anchor_a,
        b2Vec2 local_anchor_b,
        f32 reference_angle,
        bool enable_limit,
        f32 lower_angle,
        f32 upper_angle,
        bool enable_motor,
        f32 motor_speed,
        f32 max_motor_torque
);
EXPORT void RevoluteJointDef_initialize(b2RevoluteJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor);
EXPORT b2Joint* RevoluteJoint_as_joint(b2RevoluteJoint* self);
EXPORT b2RevoluteJoint* Joint_as_revolute_joint(b2Joint* self);
EXPORT const b2Vec2* RevoluteJoint_get_local_anchor_a(const b2RevoluteJoint* self);
EXPORT const b2Vec2* RevoluteJoint_get_local_anchor_b(const b2RevoluteJoint* self);
EXPORT f32 RevoluteJoint_get_reference_angle(const b2RevoluteJoint* self);
EXPORT f32 RevoluteJoint_get_joint_angle(const b2RevoluteJoint* self);
EXPORT f32 RevoluteJoint_get_joint_speed(const b2RevoluteJoint* self);
EXPORT bool RevoluteJoint_is_limit_enabled(const b2RevoluteJoint* self);
EXPORT void RevoluteJoint_enable_limit(b2RevoluteJoint* self, bool flag);
EXPORT f32 RevoluteJoint_get_lower_limit(const b2RevoluteJoint* self);
EXPORT f32 RevoluteJoint_get_upper_limit(const b2RevoluteJoint* self);
EXPORT void RevoluteJoint_set_limits(b2RevoluteJoint* self, f32 lower, f32 upper);
EXPORT bool RevoluteJoint_is_motor_enabled(const b2RevoluteJoint* self);
EXPORT void RevoluteJoint_enable_motor(b2RevoluteJoint* self, bool flag);
EXPORT void RevoluteJoint_set_motor_speed(b2RevoluteJoint* self, f32 speed);
EXPORT f32 RevoluteJoint_get_motor_speed(const b2RevoluteJoint* self);
EXPORT void RevoluteJoint_set_max_motor_torque(b2RevoluteJoint* self, f32 torque);
EXPORT f32 RevoluteJoint_get_max_motor_torque(const b2RevoluteJoint* self);
EXPORT f32 RevoluteJoint_get_motor_torque(const b2RevoluteJoint* self, f32 inv_dt);