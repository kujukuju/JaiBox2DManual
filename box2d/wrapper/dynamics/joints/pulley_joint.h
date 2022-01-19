EXPORT b2Joint* World_create_pulley_joint(
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
);
EXPORT void PulleyJointDef_initialize(b2PulleyJointDef* self,
                               b2Body* body_a, b2Body* body_b,
                               const b2Vec2* ground_anchor_a,
                               const b2Vec2* ground_anchor_b,
                               const b2Vec2* anchor_a,
                               const b2Vec2* anchor_b,
                               float ratio);
EXPORT b2Joint* PulleyJoint_as_joint(b2PulleyJoint* self);
EXPORT b2PulleyJoint* Joint_as_pulley_joint(b2Joint* self);
EXPORT b2Vec2 PulleyJoint_get_ground_anchor_a(const b2PulleyJoint* self);
EXPORT b2Vec2 PulleyJoint_get_ground_anchor_b(const b2PulleyJoint* self);
EXPORT float PulleyJoint_get_length_a(const b2PulleyJoint* self);
EXPORT float PulleyJoint_get_length_b(const b2PulleyJoint* self);
EXPORT float PulleyJoint_get_ratio(const b2PulleyJoint* self);
EXPORT float PulleyJoint_get_current_length_a(const b2PulleyJoint* self);
EXPORT float PulleyJoint_get_current_length_b(const b2PulleyJoint* self);