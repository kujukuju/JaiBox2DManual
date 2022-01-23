#include "collision/shapes/shape.h"

b2Shape* Shape_clone(b2Shape* self, b2BlockAllocator* allocator) {
    return self->Clone(allocator);
}

b2Shape::Type Shape_get_type(b2Shape* self) {
    return self->GetType();
}

int32 Shape_get_child_count(b2Shape* self) {
    return self->GetChildCount();
}

bool Shape_test_point(b2Shape* self, const b2Transform* xf, const b2Vec2* p) {
    return self->TestPoint(*xf, *p);
}

bool Shape_ray_cast(b2Shape* self, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex) {
    return self->RayCast(output, *input, *transform, childIndex);
}

void Shape_compute_aabb(b2Shape* self, b2AABB* aabb, const b2Transform* xf, int32 childIndex) {
    self->ComputeAABB(aabb, *xf, childIndex);
}

void Shape_compute_mass(b2Shape* self, b2MassData* massData, float density) {
    self->ComputeMass(massData, density);
}
