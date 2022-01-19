#include "collision/shapes/polygon_shape.h"

b2PolygonShape* PolygonShape_new() {
    return new b2PolygonShape();
}
void PolygonShape_drop(b2PolygonShape* self) {
    delete self;
}

b2Shape* PolygonShape_as_shape(b2PolygonShape* self) {
    return static_cast<b2Shape*>(self);
}
b2PolygonShape* Shape_as_polygon_shape(b2Shape* self) {
    return static_cast<b2PolygonShape*>(self);
}

void PolygonShape_set(b2PolygonShape* self,
                      const b2Vec2* points, int32_t count) {
    self->Set(points, count);
}
void PolygonShape_set_as_box(b2PolygonShape* self, float hw, float hh) {
    self->SetAsBox(hw, hh);
}
void PolygonShape_set_as_oriented_box(b2PolygonShape* self,
                                      float hw, float hh,
                                      const b2Vec2* center,
                                      float angle) {
    self->SetAsBox(hw, hh, *center, angle);
}
int32_t PolygonShape_get_vertex_count(const b2PolygonShape* self) {
    return self->GetVertexCount();
}
const b2Vec2* PolygonShape_get_vertex(const b2PolygonShape* self,
                                      int32_t index) {
    return &self->GetVertex(index);
}
bool PolygonShape_validate(const b2PolygonShape* self) {
    return self->Validate();
}