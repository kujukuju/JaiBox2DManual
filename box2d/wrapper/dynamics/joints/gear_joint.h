EXPORT b2Joint* World_create_gear_joint(
        b2World* self,
        bool collide_connected,
        b2Joint* joint_a,
        b2Joint* joint_b,
        f32 ratio
);
EXPORT b2Joint* GearJoint_as_joint(b2GearJoint* self);
EXPORT b2GearJoint* Joint_as_gear_joint(b2Joint* self);
EXPORT b2Joint* GearJoint_get_joint_1(b2GearJoint* self);
EXPORT b2Joint* GearJoint_get_joint_2(b2GearJoint* self);
EXPORT void GearJoint_set_ratio(b2GearJoint* self, f32 ratio);
EXPORT f32 GearJoint_get_ratio(const b2GearJoint* self);