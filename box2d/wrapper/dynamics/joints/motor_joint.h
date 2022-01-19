EXPORT b2Joint* World_create_motor_joint(
        b2World* self,
        b2Body* body_a,
        b2Body* body_b,
        bool collide_connected,
        b2Vec2 linear_offset,
        float angular_offset,
        float max_force,
        float max_torque,
        float correction_factor
);
EXPORT void MotorJointDef_initialize(b2MotorJointDef* self,
                              b2Body* body_a, b2Body* body_b);
EXPORT b2Joint* MotorJoint_as_joint(b2MotorJoint* self);
EXPORT b2MotorJoint* Joint_as_motor_joint(b2Joint* self);
EXPORT void MotorJoint_set_linear_offset(b2MotorJoint* self,
                                  const b2Vec2* offset);
EXPORT const b2Vec2* MotorJoint_get_linear_offset(const b2MotorJoint* self);
EXPORT void MotorJoint_set_angular_offset(b2MotorJoint* self, float offset);
EXPORT float MotorJoint_get_angular_offset(const b2MotorJoint* self);
EXPORT void MotorJoint_set_max_force(b2MotorJoint* self, float force);
EXPORT float MotorJoint_get_max_force(const b2MotorJoint* self);
EXPORT void MotorJoint_set_max_torque(b2MotorJoint* self, float torque);
EXPORT float MotorJoint_get_max_torque(const b2MotorJoint* self);
EXPORT void MotorJoint_set_correction_factor(b2MotorJoint* self, float factor);
EXPORT float MotorJoint_get_correction_factor(const b2MotorJoint* self);