EXPORT b2Shape* Shape_clone(b2Shape* shape, b2BlockAllocator* allocator);
EXPORT b2Shape::Type Shape_get_type(b2Shape* shape);
EXPORT int32 Shape_get_child_count(b2Shape* shape);
EXPORT bool Shape_test_point(b2Shape* shape, const b2Transform* xf, const b2Vec2* p);
EXPORT bool Shape_ray_cast(b2Shape* shape, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex);
EXPORT void Shape_compute_aabb(b2Shape* shape, b2AABB* aabb, const b2Transform* xf, int32 childIndex);
EXPORT void Shape_compute_mass(b2Shape* shape, b2MassData* massData, float density);