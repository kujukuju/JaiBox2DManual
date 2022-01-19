EXPORT b2Joint* World_create_weld_joint(
        b2World* world,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 local_anchor_a,
        b2Vec2 local_anchor_b,
        f32 reference_angle,
        f32 frequency,
        f32 damping_ratio
);
EXPORT void WeldJointDef_initialize(b2WeldJointDef* self,
                             b2Body* body_a, b2Body* body_b,
                             const b2Vec2* anchor);
EXPORT b2Joint* WeldJoint_as_joint(b2WeldJoint* self);
EXPORT b2WeldJoint* Joint_as_weld_joint(b2Joint* self);
EXPORT const b2Vec2* WeldJoint_get_local_anchor_a(const b2WeldJoint* self);
EXPORT const b2Vec2* WeldJoint_get_local_anchor_b(const b2WeldJoint* self);
EXPORT f32 WeldJoint_get_reference_angle(const b2WeldJoint* self);
EXPORT void WeldJoint_set_frequency(b2WeldJoint* self, f32 frequency);
EXPORT f32 WeldJoint_get_frequency(const b2WeldJoint* self);
EXPORT void WeldJoint_set_damping_ratio(b2WeldJoint* self, f32 ratio);
EXPORT f32 WeldJoint_get_damping_ratio(const b2WeldJoint* self);