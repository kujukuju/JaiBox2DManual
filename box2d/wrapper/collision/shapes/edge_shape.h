EXPORT b2EdgeShape* EdgeShape_new();
EXPORT void EdgeShape_drop(b2EdgeShape* self);
EXPORT b2Shape* EdgeShape_as_shape(b2EdgeShape* self);
EXPORT b2EdgeShape* Shape_as_edge_shape(b2Shape* self);
EXPORT void EdgeShape_set(b2EdgeShape* self,
                   const b2Vec2* v1, const b2Vec2* v2);
EXPORT b2Vec2 EdgeShape_get_v1(const b2EdgeShape* self);
EXPORT void EdgeShape_set_v1(b2EdgeShape* self, b2Vec2 v1);
EXPORT b2Vec2 EdgeShape_get_v2(const b2EdgeShape* self);
EXPORT void EdgeShape_set_v2(b2EdgeShape* self, b2Vec2 v2);
EXPORT bool EdgeShape_get_v0(const b2EdgeShape* self, b2Vec2* v0);
EXPORT void EdgeShape_set_v0(b2EdgeShape* self, const b2Vec2* v0);
EXPORT bool EdgeShape_get_v3(const b2EdgeShape* self, b2Vec2* v3);
EXPORT void EdgeShape_set_v3(b2EdgeShape* self, const b2Vec2* v3);