#include "collision/shapes/edge_shape.h"

b2EdgeShape* EdgeShape_new() {
    return new b2EdgeShape();
}

b2EdgeShape EdgeShape_create() {
    return b2EdgeShape();
}

void EdgeShape_set_one_sided(b2EdgeShape* self, const b2Vec2* v0, const b2Vec2* v1, const b2Vec2* v2, const b2Vec2* v3) {
    self->SetOneSided(*v0, *v1, *v2, *v3);
}

void EdgeShape_set_two_sided(b2EdgeShape* self, const b2Vec2* v1, const b2Vec2* v2) {
    self->SetTwoSided(*v1, *v2);
}

b2Shape* EdgeShape_clone(b2EdgeShape* self, b2BlockAllocator* allocator) {
    return self->Clone(allocator);
}

int32 EdgeShape_get_child_count(b2EdgeShape* self) {
    return self->GetChildCount();
}

bool EdgeShape_test_point(b2EdgeShape* self, const b2Transform* transform, const b2Vec2* p) {
    return self->TestPoint(*transform, *p);
}

void EdgeShape_compute_distance(b2EdgeShape* self, const b2Transform* xf, const b2Vec2* p, float32* distance, b2Vec2* normal, int32 childIndex) {
    self->ComputeDistance(*xf, *p, distance, normal, childIndex);
}

bool EdgeShape_ray_cast(b2EdgeShape* self, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex) {
    return self->RayCast(output, *input, *transform, childIndex);
}

void EdgeShape_compute_aabb(b2EdgeShape* self, b2AABB* aabb, const b2Transform* transform, int32 childIndex) {
    self->ComputeAABB(aabb, *transform, childIndex);
}

void EdgeShape_compute_mass(b2EdgeShape* self, b2MassData* massData, float density) {
    self->ComputeMass(massData, density);
}
