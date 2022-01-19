EXPORT b2Joint* World_create_rope_joint(
        b2World* world,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 local_anchor_a,
        b2Vec2 local_anchor_b,
        f32 max_length
);
EXPORT b2Joint* RopeJoint_as_joint(b2RopeJoint* self);
EXPORT b2RopeJoint* Joint_as_rope_joint(b2Joint* self);
EXPORT const b2Vec2* RopeJoint_get_local_anchor_a(const b2RopeJoint* self);
EXPORT const b2Vec2* RopeJoint_get_local_anchor_b(const b2RopeJoint* self);
EXPORT void RopeJoint_set_max_length(b2RopeJoint* self, f32 length);
EXPORT f32 RopeJoint_get_max_length(const b2RopeJoint* self);
EXPORT b2LimitState RopeJoint_get_limit_state(const b2RopeJoint* self);