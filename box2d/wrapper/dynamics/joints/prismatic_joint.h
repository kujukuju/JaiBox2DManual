EXPORT b2Joint* World_create_prismatic_joint(
        b2World* self,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 local_anchor_a,
        b2Vec2 local_anchor_b,
        b2Vec2 local_axis_a,
        f32 reference_angle,
        bool enable_limit,
        f32 lower_translation,
        f32 upper_translation,
        bool enable_motor,
        f32 max_motor_force,
        f32 motor_speed
);
EXPORT void PrismaticJointDef_initialize(b2PrismaticJointDef* self,
                                  b2Body* body_a, b2Body* body_b,
                                  const b2Vec2* anchor,
                                  const b2Vec2* axis);
EXPORT b2Joint* PrismaticJoint_as_joint(b2PrismaticJoint* self);
EXPORT b2PrismaticJoint* Joint_as_prismatic_joint(b2Joint* self);
EXPORT const b2Vec2* PrismaticJoint_get_local_anchor_a(const b2PrismaticJoint* self);
EXPORT const b2Vec2* PrismaticJoint_get_local_anchor_b(const b2PrismaticJoint* self);
EXPORT const b2Vec2* PrismaticJoint_get_local_axis_a(const b2PrismaticJoint* self);
EXPORT f32 PrismaticJoint_get_reference_angle(const b2PrismaticJoint* self);
EXPORT f32 PrismaticJoint_get_joint_translation(const b2PrismaticJoint* self);
EXPORT f32 PrismaticJoint_get_joint_speed(const b2PrismaticJoint* self);
EXPORT bool PrismaticJoint_is_limit_enabled(const b2PrismaticJoint* self);
EXPORT void PrismaticJoint_enable_limit(b2PrismaticJoint* self, bool flag);
EXPORT f32 PrismaticJoint_get_lower_limit(const b2PrismaticJoint* self);
EXPORT f32 PrismaticJoint_get_upper_limit(const b2PrismaticJoint* self);
EXPORT void PrismaticJoint_set_limits(b2PrismaticJoint* self, f32 lower, f32 upper);
EXPORT bool PrismaticJoint_is_motor_enabled(const b2PrismaticJoint* self);
EXPORT void PrismaticJoint_enable_motor(b2PrismaticJoint* self, bool flag);
EXPORT void PrismaticJoint_set_motor_speed(b2PrismaticJoint* self, f32 speed);
EXPORT f32 PrismaticJoint_get_motor_speed(const b2PrismaticJoint* self);
EXPORT void PrismaticJoint_set_max_motor_force(b2PrismaticJoint* self, f32 force);
EXPORT f32 PrismaticJoint_get_max_motor_force(const b2PrismaticJoint* self);
EXPORT f32 PrismaticJoint_get_motor_force(const b2PrismaticJoint* self, f32 inv_dt);