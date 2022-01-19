EXPORT b2ChainShape* ChainShape_new();
EXPORT void ChainShape_drop(b2ChainShape* self);
EXPORT b2Shape* ChainShape_as_shape(b2ChainShape* self);
EXPORT b2ChainShape* Shape_as_chain_shape(b2Shape* self);
EXPORT void ChainShape_clear(b2ChainShape* self);
EXPORT void ChainShape_create_loop(b2ChainShape* self,
                            const b2Vec2* vertices,
                            i32 count);
EXPORT void ChainShape_create_chain(b2ChainShape* self,
                             const b2Vec2* vertices,
                             i32 count);
EXPORT const b2Vec2* ChainShape_get_vertices_const(const b2ChainShape* self);
EXPORT i32 ChainShape_get_vertex_count(const b2ChainShape* self);
EXPORT bool ChainShape_get_prev_vertex(const b2ChainShape* self, b2Vec2* prev);
EXPORT void ChainShape_set_prev_vertex(b2ChainShape* self, const b2Vec2* prev);
EXPORT bool ChainShape_get_next_vertex(const b2ChainShape* self, b2Vec2* next);
EXPORT void ChainShape_set_next_vertex(b2ChainShape* self, const b2Vec2* next);
EXPORT void ChainShape_get_child_edge(const b2ChainShape* self,
                               b2EdgeShape* edge, i32 index);