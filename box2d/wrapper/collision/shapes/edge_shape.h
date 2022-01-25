EXPORT b2EdgeShape* EdgeShape_new();
EXPORT b2EdgeShape EdgeShape_create();
EXPORT void EdgeShape_set_one_sided(b2EdgeShape* self, const b2Vec2* v0, const b2Vec2* v1,const b2Vec2* v2, const b2Vec2* v3);
EXPORT void EdgeShape_set_two_sided(b2EdgeShape* self, const b2Vec2* v1, const b2Vec2* v2);
EXPORT b2Shape* EdgeShape_clone(b2EdgeShape* self, b2BlockAllocator* allocator);
EXPORT int32 EdgeShape_get_child_count(b2EdgeShape* self);
EXPORT bool EdgeShape_test_point(b2EdgeShape* self, const b2Transform* transform, const b2Vec2* p);
EXPORT void EdgeShape_compute_distance(b2EdgeShape* self, const b2Transform* xf, const b2Vec2* p, float32* distance, b2Vec2* normal, int32 childIndex);
EXPORT bool EdgeShape_ray_cast(b2EdgeShape* self, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex);
EXPORT void EdgeShape_compute_aabb(b2EdgeShape* self, b2AABB* aabb, const b2Transform* transform, int32 childIndex);
EXPORT void EdgeShape_compute_mass(b2EdgeShape* self, b2MassData* massData, float density);
EXPORT void EdgeShape_set(b2EdgeShape* self, float32 vx1, float32 vy1, float32 vx2, float32 vy2);