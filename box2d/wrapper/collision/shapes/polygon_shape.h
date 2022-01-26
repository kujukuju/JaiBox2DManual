EXPORT b2PolygonShape* PolygonShape_new();
EXPORT b2PolygonShape PolygonShape_create();
EXPORT b2Shape* PolygonShape_clone(b2PolygonShape* self, b2BlockAllocator* allocator);
EXPORT int32 PolygonShape_get_child_count(b2PolygonShape* self);
EXPORT void PolygonShape_set(b2PolygonShape* self, const b2Vec2* points, int32 count);
EXPORT void PolygonShape_set_as_box(b2PolygonShape* self, float hx, float hy);
// TODO maybe dont extern c so I can overload this? will that work?
EXPORT void PolygonShape_set_as_box_angled(b2PolygonShape* self, float hx, float hy, const b2Vec2* center, float angle);
EXPORT bool PolygonShape_test_point(b2PolygonShape* self, const b2Transform* transform, const b2Vec2* p);
EXPORT void PolygonShape_compute_distance(b2PolygonShape* self, const b2Transform* xf, const b2Vec2* p, float32* distance, b2Vec2* normal, int32 childIndex);
EXPORT bool PolygonShape_ray_cast(b2PolygonShape* self, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex);
EXPORT void PolygonShape_compute_aabb(b2PolygonShape* self, b2AABB* aabb, const b2Transform* transform, int32 childIndex);
EXPORT void PolygonShape_compute_mass(b2PolygonShape* self, b2MassData* massData, float density);
EXPORT bool PolygonShape_validate(b2PolygonShape* self);
EXPORT void PolygonShape_set_centroid(b2PolygonShape* self, float32 x, float32 y);