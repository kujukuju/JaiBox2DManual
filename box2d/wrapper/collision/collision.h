EXPORT void WorldManifold_Initialize(b2WorldManifold* self,
                              const b2Manifold* manifold,
                              const b2Transform* xf_a, float radius_a,
                              const b2Transform* xf_b, float radius_b);
EXPORT void get_point_states(b2PointState* s1, b2PointState* s2,
                      const b2Manifold* m1, const b2Manifold* m2);
EXPORT bool test_overlap(const b2Shape* shape_a, int32_t index_a,
                  const b2Shape* shape_b, int32_t index_b,
                  const b2Transform* xf_a, const b2Transform* xf_b);
EXPORT void DistanceProxy_set(b2DistanceProxy* self,
                       const b2Shape* shape, int32_t index);
EXPORT void distance(b2DistanceOutput* output,
              b2SimplexCache* cache,
              const b2DistanceInput* input);
EXPORT void time_of_impact(b2TOIOutput* output, const b2TOIInput* input);
EXPORT b2Manifold* Contact_get_manifold(b2Contact* self);
EXPORT void Contact_get_world_manifold(const b2Contact* self,
                                b2WorldManifold* world_manifold);
EXPORT bool Contact_is_touching(const b2Contact* self);
EXPORT bool Contact_is_enabled(const b2Contact* self);
EXPORT b2Contact* Contact_get_next(b2Contact* self);
EXPORT b2Fixture* Contact_get_fixture_a(b2Contact* self);
EXPORT int32_t Contact_get_child_index_a(const b2Contact* self);
EXPORT b2Fixture* Contact_get_fixture_b(b2Contact* self);
EXPORT int32_t Contact_get_child_index_b(const b2Contact* self);
EXPORT void Contact_set_friction(b2Contact* self, float friction);
EXPORT float Contact_get_friction(const b2Contact* self);
EXPORT void Contact_reset_friction(b2Contact* self);
EXPORT void Contact_set_restitution(b2Contact* self, float restitution);
EXPORT float Contact_get_restitution(const b2Contact* self);
EXPORT void Contact_reset_restitution(b2Contact* self);
EXPORT void Contact_set_tangent_speed(b2Contact* self, float speed);
EXPORT float Contact_get_tangent_speed(const b2Contact* self);
EXPORT void Contact_evaluate_virtual(b2Contact* self, b2Manifold* manifold,
                              const b2Transform* xf_a,
                              const b2Transform* xf_b);