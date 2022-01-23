EXPORT b2ChainShape* ChainShape_new();
EXPORT b2ChainShape ChainShape_create();
EXPORT void ChainShape_clear(b2ChainShape* self);
EXPORT void ChainShape_create_loop(b2ChainShape* self,
                            const b2Vec2* vertices,
                            int32_t count);
EXPORT void ChainShape_create_chain(b2ChainShape* self,
                             const b2Vec2* vertices,
                             int32_t count,
                             const b2Vec2 prevVertex,
                             const b2Vec2 nextVertex);
EXPORT b2Shape* ChainShape_clone(b2ChainShape* self, b2BlockAllocator* allocator);
EXPORT int32_t ChainShape_get_child_count(b2ChainShape* self);
EXPORT void ChainShape_get_child_edge(b2ChainShape* self, b2EdgeShape* edge, int32_t index);
EXPORT bool ChainShape_test_point(b2ChainShape* self, b2Transform* transform, b2Vec2* p);
EXPORT bool ChainShape_ray_cast(b2ChainShape* self,
                                b2RayCastOutput* output,
                                b2RayCastInput* input,
					            b2Transform* transform,
                                int32 childIndex);
EXPORT void ChainShape_compute_aabb(b2ChainShape* self,
                                    b2AABB* aabb,
                                    const b2Transform* transform,
                                    int32 childIndex);
EXPORT void ChainShape_compute_mass(b2ChainShape* self,
                                    b2MassData* massData,
                                    float density);