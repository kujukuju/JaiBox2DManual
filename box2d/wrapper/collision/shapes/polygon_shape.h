EXPORT b2PolygonShape* PolygonShape_new();
EXPORT void PolygonShape_drop(b2PolygonShape* self);
EXPORT b2Shape* PolygonShape_as_shape(b2PolygonShape* self);
EXPORT b2PolygonShape* Shape_as_polygon_shape(b2Shape* self);
EXPORT void PolygonShape_set(b2PolygonShape* self,
                      const b2Vec2* points, i32 count);
EXPORT void PolygonShape_set_as_box(b2PolygonShape* self, f32 hw, f32 hh);
EXPORT void PolygonShape_set_as_oriented_box(b2PolygonShape* self,
                                      f32 hw, f32 hh,
                                      const b2Vec2* center,
                                      f32 angle);
EXPORT i32 PolygonShape_get_vertex_count(const b2PolygonShape* self);
EXPORT const b2Vec2* PolygonShape_get_vertex(const b2PolygonShape* self,
                                      i32 index);
EXPORT bool PolygonShape_validate(const b2PolygonShape* self);