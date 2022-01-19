EXPORT b2Joint* World_create_distance_joint(
        b2World* self,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 local_anchor_a,
        b2Vec2 local_anchor_b,
        f32 length,
        f32 frequency,
        f32 damping_ratio
);
EXPORT void DistanceJointDef_initialize(b2DistanceJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor_a,
                                 const b2Vec2* anchor_b);
EXPORT b2Joint* DistanceJoint_as_joint(b2DistanceJoint* self);
EXPORT b2DistanceJoint* Joint_as_distance_joint(b2Joint* self);
EXPORT const b2Vec2* DistanceJoint_get_local_anchor_a(
        const b2DistanceJoint* self);
EXPORT const b2Vec2* DistanceJoint_get_local_anchor_b(
        const b2DistanceJoint* self);
EXPORT void DistanceJoint_set_length(b2DistanceJoint* self, f32 length);
EXPORT f32 DistanceJoint_get_length(const b2DistanceJoint* self);
EXPORT void DistanceJoint_set_frequency(b2DistanceJoint* self, f32 hz);
EXPORT f32 DistanceJoint_get_frequency(const b2DistanceJoint* self);
EXPORT void DistanceJoint_set_damping_ratio(b2DistanceJoint* self,
                                     f32 ratio);
EXPORT f32 DistanceJoint_get_damping_ratio(const b2DistanceJoint* self);