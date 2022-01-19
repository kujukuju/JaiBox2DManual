EXPORT b2Joint* World_create_friction_joint(
        b2World* self,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 local_anchor_a,
        b2Vec2 local_anchor_b,
        float max_force,
        float max_torque
);
EXPORT void FrictionJointDef_initialize(b2FrictionJointDef* self,
                                 b2Body* body_a, b2Body* body_b,
                                 const b2Vec2* anchor);
EXPORT b2Joint* FrictionJoint_as_joint(b2FrictionJoint* self);
EXPORT b2FrictionJoint* Joint_as_friction_joint(b2Joint* self);
EXPORT const b2Vec2* FrictionJoint_get_local_anchor_a(const b2FrictionJoint* self);
EXPORT const b2Vec2* FrictionJoint_get_local_anchor_b(const b2FrictionJoint* self);
EXPORT void FrictionJoint_set_max_force(b2FrictionJoint* self, float force);
EXPORT float FrictionJoint_get_max_force(const b2FrictionJoint* self);
EXPORT void FrictionJoint_set_max_torque(b2FrictionJoint* self, float torque);
EXPORT float FrictionJoint_get_max_torque(const b2FrictionJoint* self);