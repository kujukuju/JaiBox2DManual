EXPORT b2CircleShape* CircleShape_new();
EXPORT void CircleShape_drop(b2CircleShape* self);
EXPORT b2CircleShape CircleShape_create();
EXPORT b2Shape* CircleShape_as_shape(b2CircleShape* self);
EXPORT b2CircleShape* Shape_as_circle_shape(b2Shape* self);
EXPORT int32_t CircleShape_get_support(const b2CircleShape* self,
                            const b2Vec2* d);
EXPORT const b2Vec2* CircleShape_get_support_vertex(const b2CircleShape* self,
                                             const b2Vec2* d);
EXPORT int32_t CircleShape_get_vertex_count(const b2CircleShape* self);
EXPORT const b2Vec2* CircleShape_get_vertex(const b2CircleShape* self,
                                     int32_t index);
EXPORT b2Vec2 CircleShape_get_pos(const b2CircleShape* self);
EXPORT void CircleShape_set_pos(b2CircleShape* self, b2Vec2 pos);