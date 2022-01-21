EXPORT b2PolygonShape* PolygonShape_new();
EXPORT void PolygonShape_drop(b2PolygonShape* self);
EXPORT b2PolygonShape PolygonShape_create();
EXPORT b2Shape* PolygonShape_as_shape(b2PolygonShape* self);
EXPORT b2PolygonShape* Shape_as_polygon_shape(b2Shape* self);
EXPORT void PolygonShape_set(b2PolygonShape* self,
                      const b2Vec2* points, int32_t count);
EXPORT void PolygonShape_set_as_box(b2PolygonShape* self, float hw, float hh);
EXPORT void PolygonShape_set_as_oriented_box(b2PolygonShape* self,
                                      float hw, float hh,
                                      const b2Vec2* center,
                                      float angle);
EXPORT int32_t PolygonShape_get_vertex_count(const b2PolygonShape* self);
EXPORT const b2Vec2* PolygonShape_get_vertex(const b2PolygonShape* self,
                                      int32_t index);
EXPORT bool PolygonShape_validate(const b2PolygonShape* self);