EXPORT int32_t Fixture_get_type(const b2Fixture* self);
EXPORT b2Shape* Fixture_get_shape(b2Fixture* self);
EXPORT void Fixture_set_sensor(b2Fixture* self, bool flag);
EXPORT bool Fixture_is_sensor(const b2Fixture* self);
EXPORT void Fixture_set_filter_data(b2Fixture* self, const b2Filter* filter);
EXPORT const b2Filter* Fixture_get_filter_data(const b2Fixture* self);
EXPORT void Fixture_refilter(b2Fixture* self);
EXPORT b2Body* Fixture_get_body(b2Fixture* self);
EXPORT b2Fixture* Fixture_get_next(b2Fixture* self);
EXPORT void* Fixture_get_user_data(const b2Fixture* self);
EXPORT void Fixture_set_user_data(b2Fixture* self, void* data);
EXPORT bool Fixture_test_point(const b2Fixture* self, const b2Vec2* p);
EXPORT bool Fixture_ray_cast(const b2Fixture* self,
                      b2RayCastOutput* output,
                      const b2RayCastInput* input,
                      int32_t child_id);
EXPORT void Fixture_get_mass_data(const b2Fixture* self, b2MassData* data);
EXPORT void Fixture_set_density(b2Fixture* self, float density);
EXPORT float Fixture_get_density(const b2Fixture* self);
EXPORT float Fixture_get_friction(const b2Fixture* self);
EXPORT void Fixture_set_friction(b2Fixture* self, float friction);
EXPORT float Fixture_get_restitution(const b2Fixture* self);
EXPORT void Fixture_set_restitution(b2Fixture* self, float restitution);
EXPORT const b2AABB* Fixture_get_aabb(const b2Fixture* self, int32_t child_id);
EXPORT void Fixture_dump(b2Fixture* self, int32_t body_id);