EXPORT b2CircleShape* CircleShape_new();
EXPORT b2CircleShape CircleShape_create();
EXPORT bool CircleShape_test_point(b2CircleShape* self, b2Transform* transform, b2Vec2* p);
EXPORT bool CircleShape_ray_cast(b2CircleShape* self,
                                 b2RayCastOutput* output,
                                 const b2RayCastInput* input,
				                 const b2Transform* transform,
                                 int32 childIndex);
EXPORT void CircleShape_compute_aabb(b2CircleShape* self,
                                     b2AABB* aabb,
                                     const b2Transform* transform,
                                     int32 childIndex);
EXPORT void CircleShape_compute_mass(b2CircleShape* self,
                                     b2MassData* massData,
                                     float density);