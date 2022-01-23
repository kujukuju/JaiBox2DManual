#include "collision/shapes/polygon_shape.h"

b2PolygonShape* PolygonShape_new() {
    return new b2PolygonShape();
}

b2PolygonShape PolygonShape_create() {
    return b2PolygonShape();
}

b2Shape* PolygonShape_clone(b2PolygonShape* self, b2BlockAllocator* allocator) {
    return self->Clone(allocator);
}

int32 PolygonShape_get_child_count(b2PolygonShape* self) {
    return self->GetChildCount();
}

void PolygonShape_set(b2PolygonShape* self, const b2Vec2* points, int32 count) {
    self->Set(points, count);
}

void PolygonShape_set_as_box(b2PolygonShape* self, float hx, float hy) {
    self->SetAsBox(hx, hy);
}

void PolygonShape_set_as_box_angled(b2PolygonShape* self, float hx, float hy, const b2Vec2* center, float angle) {
    self->SetAsBox(hx, hy, *center, angle);
}

bool PolygonShape_test_point(b2PolygonShape* self, const b2Transform* transform, const b2Vec2* p) {
    return self->TestPoint(*transform, *p);
}

bool PolygonShape_ray_cast(b2PolygonShape* self, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex) {
    return self->RayCast(output, *input, *transform, childIndex);
}

void PolygonShape_compute_aabb(b2PolygonShape* self, b2AABB* aabb, const b2Transform* transform, int32 childIndex) {
    self->ComputeAABB(aabb, *transform, childIndex);
}

void PolygonShape_compute_mass(b2PolygonShape* self, b2MassData* massData, float density) {
    self->ComputeMass(massData, density);
}

bool PolygonShape_validate(b2PolygonShape* self) {
    return self->Validate();
}
