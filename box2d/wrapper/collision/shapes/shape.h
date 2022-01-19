EXPORT void Shape_drop_virtual(b2Shape* self);
EXPORT int32_t Shape_get_type(const b2Shape* self);
EXPORT int32_t Shape_get_child_count_virtual(const b2Shape* self);
EXPORT bool Shape_test_point_virtual(const b2Shape* self,
                              const b2Transform* xf,
                              const b2Vec2* p);
EXPORT bool Shape_ray_cast_virtual(const b2Shape* self,
                            b2RayCastOutput* output,
                            const b2RayCastInput* input,
                            const b2Transform* transform,
                            int32_t child_id);
EXPORT void Shape_compute_aabb_virtual(const b2Shape* self,
                                b2AABB* aabb,
                                const b2Transform* xf,
                                int32_t child_id);
EXPORT void Shape_compute_mass_virtual(const b2Shape* self,
                                b2MassData* data,
                                float density);
EXPORT float Shape_get_radius(const b2Shape* self);
EXPORT void Shape_set_radius(b2Shape* self, float radius);