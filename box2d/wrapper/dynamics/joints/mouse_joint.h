EXPORT b2Joint* World_create_mouse_joint(
        b2World* self,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 target,
        float max_force,
        float frequency,
        float damping_ratio
);
EXPORT b2Joint* MouseJoint_as_joint(b2MouseJoint* self);
EXPORT b2MouseJoint* Joint_as_mouse_joint(b2Joint* self);
EXPORT void MouseJoint_set_target(b2MouseJoint* self, const b2Vec2* target);
EXPORT const b2Vec2* MouseJoint_get_target(const b2MouseJoint* self);
EXPORT void MouseJoint_set_max_force(b2MouseJoint* self, float force);
EXPORT float MouseJoint_get_max_force(const b2MouseJoint* self);
EXPORT void MouseJoint_set_frequency(b2MouseJoint* self, float hz);
EXPORT float MouseJoint_get_frequency(const b2MouseJoint* self);
EXPORT void MouseJoint_set_damping_ratio(b2MouseJoint* self, float ratio);
EXPORT float MouseJoint_get_damping_ratio(const b2MouseJoint* self);