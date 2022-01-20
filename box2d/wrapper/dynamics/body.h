b2Fixture* Body_create_fixture(b2Body* self, const b2FixtureDef* def);
b2Fixture* Body_create_fast_fixture(b2Body* self,
                                    const b2Shape* shape,
                                    float density);
void Body_destroy_fixture(b2Body* self, b2Fixture* fixture);
void Body_set_transform(b2Body* self, const b2Vec2* pos, float angle);
const b2Transform* Body_get_transform(const b2Body* self);
const b2Vec2* Body_get_position(const b2Body* self);
float Body_get_angle(const b2Body* self);
const b2Vec2* Body_get_world_center(const b2Body* self);
const b2Vec2* Body_get_local_center(const b2Body* self);
void Body_set_linear_velocity(b2Body* self, const b2Vec2* v);
const b2Vec2* Body_get_linear_velocity(const b2Body* self);
void Body_set_angular_velocity(b2Body* self, float omega);
float Body_get_angular_velocity(const b2Body* self);
void Body_apply_force(b2Body* self,
                      const b2Vec2* force,
                      const b2Vec2* point,
                      bool wake);
void Body_apply_force_to_center(b2Body* self,
                                const b2Vec2* force,
                                bool wake);
void Body_apply_torque(b2Body* self, float torque, bool wake);
EXPORT void Body_apply_linear_impulse(b2Body* self,
                               const b2Vec2* impulse,
                               const b2Vec2* point,
                               bool wake);
EXPORT void Body_apply_angular_impulse(b2Body* self,
                                float impulse,
                                bool wake);
EXPORT float Body_get_mass(const b2Body* self);
EXPORT float Body_get_inertia(const b2Body* self);
EXPORT void Body_get_mass_data(const b2Body* self, b2MassData* data);
EXPORT void Body_set_mass_data(b2Body* self, const b2MassData* data);
EXPORT void Body_reset_mass_data(b2Body* self);
EXPORT b2Vec2 Body_get_world_point(const b2Body* self, const b2Vec2* local);
EXPORT b2Vec2 Body_get_world_vector(const b2Body* self, const b2Vec2* local);
EXPORT b2Vec2 Body_get_local_point(const b2Body* self, const b2Vec2* world);
EXPORT b2Vec2 Body_get_local_vector(const b2Body* self, const b2Vec2* world);
EXPORT b2Vec2 Body_get_linear_velocity_from_world_point(const b2Body* self,
                                                 const b2Vec2* point);
EXPORT b2Vec2 Body_get_linear_velocity_from_local_point(const b2Body* self,
                                                 const b2Vec2* point);
EXPORT float Body_get_linear_damping(const b2Body* self);
EXPORT void Body_set_linear_damping(b2Body* self, float damping);
EXPORT float Body_get_angular_damping(const b2Body* self);
EXPORT void Body_set_angular_damping(b2Body* self, float damping);
EXPORT float Body_get_gravity_scale(const b2Body* self);
EXPORT void Body_set_gravity_scale(b2Body* self, float scale);
EXPORT void Body_set_type(b2Body* self, int32_t type);
EXPORT int32_t Body_get_type(const b2Body* self);
EXPORT void Body_set_bullet(b2Body* self, bool flag);
EXPORT bool Body_is_bullet(const b2Body* self);
EXPORT void Body_set_sleeping_allowed(b2Body* self, bool flag);
EXPORT bool Body_is_sleeping_allowed(const b2Body* self);
EXPORT void Body_set_awake(b2Body* self, bool flag);
EXPORT bool Body_is_awake(const b2Body* self);
EXPORT void Body_set_active(b2Body* self, bool flag);
EXPORT bool Body_is_active(const b2Body* self);
EXPORT void Body_set_fixed_rotation(b2Body* self, bool flag);
EXPORT bool Body_is_fixed_rotation(const b2Body* self);
EXPORT b2Fixture* Body_get_fixture_list(b2Body* self);
EXPORT b2JointEdge* Body_get_joint_list(b2Body* self);
EXPORT b2ContactEdge* Body_get_contact_list(b2Body* self);
EXPORT b2Body* Body_get_next(b2Body* self);
EXPORT void* Body_get_user_data(const b2Body* self);
EXPORT void Body_set_user_data(b2Body* self, void* data);
EXPORT b2World* Body_get_world(b2Body* self);
EXPORT void Body_dump(b2Body* self);