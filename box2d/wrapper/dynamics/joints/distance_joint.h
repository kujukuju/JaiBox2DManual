EXPORT b2DistanceJointDef* DistanceJointDef_new();
EXPORT b2DistanceJointDef DistanceJointDef_create();
EXPORT void DistanceJointDef_initialize(b2DistanceJointDef* self, b2Body* bodyA, b2Body* bodyB, const b2Vec2* anchorA, const b2Vec2* anchorB);
EXPORT b2Vec2 DistanceJoint_get_anchor_a(b2DistanceJoint* self);
EXPORT b2Vec2 DistanceJoint_get_anchor_b(b2DistanceJoint* self);
EXPORT b2Vec2 DistanceJoint_get_reaction_force(b2DistanceJoint* self, float inv_dt);
EXPORT float DistanceJoint_get_reaction_torque(b2DistanceJoint* self, float inv_dt);
EXPORT const b2Vec2* DistanceJoint_get_local_anchor_a(b2DistanceJoint* self);
EXPORT const b2Vec2* DistanceJoint_get_local_anchor_b(b2DistanceJoint* self);
EXPORT float DistanceJoint_get_length(b2DistanceJoint* self);
EXPORT float DistanceJoint_set_length(b2DistanceJoint* self, float length);
EXPORT float DistanceJoint_get_min_length(b2DistanceJoint* self);
EXPORT float DistanceJoint_set_min_length(b2DistanceJoint* self, float minLength);
EXPORT float DistanceJoint_get_max_length(b2DistanceJoint* self);
EXPORT float DistanceJoint_set_max_length(b2DistanceJoint* self, float maxLength);
EXPORT float DistanceJoint_get_current_length(b2DistanceJoint* self);
EXPORT void DistanceJoint_set_stiffness(b2DistanceJoint* self, float stiffness);
EXPORT float DistanceJoint_get_stiffness(b2DistanceJoint* self);
EXPORT void DistanceJoint_set_damping(b2DistanceJoint* self, float damping);
EXPORT float DistanceJoint_get_damping(b2DistanceJoint* self);
EXPORT void DistanceJoint_dump(b2DistanceJoint* self);
EXPORT void DistanceJoint_draw(b2DistanceJoint* self, b2Draw* draw);