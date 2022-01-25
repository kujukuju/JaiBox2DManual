#include "collision/shapes/chain_shape.h"

b2ChainShape* ChainShape_new() {
    return new b2ChainShape();
}

b2ChainShape ChainShape_create() {
    return b2ChainShape();
}

void ChainShape_clear(b2ChainShape* self) {
    self->Clear();
}

void ChainShape_create_loop(b2ChainShape* self, const b2Vec2* vertices, int32_t count) {
    self->CreateLoop(vertices, count);
}

void ChainShape_create_chain(b2ChainShape* self, const b2Vec2* vertices, int32_t count, const b2Vec2 prevVertex, const b2Vec2 nextVertex) {
    self->CreateChain(vertices, count, prevVertex, nextVertex);
}

b2Shape* ChainShape_clone(b2ChainShape* self, b2BlockAllocator* allocator) {
    return self->Clone(allocator);
}

int32_t ChainShape_get_child_count(b2ChainShape* self) {
    return self->GetChildCount();
}

void ChainShape_get_child_edge(b2ChainShape* self, b2EdgeShape* edge, int32_t index) {
    self->GetChildEdge(edge, index);
}

void ChainShape_compute_distance(b2ChainShape* self, b2Transform* xf, const b2Vec2* p, float32* distance, b2Vec2* normal, int32 childIndex) {
    self->ComputeDistance(*xf, *p, distance, normal, childIndex);
}

bool ChainShape_test_point(b2ChainShape* self, b2Transform* transform, b2Vec2* p) {
    return self->TestPoint(*transform, *p);
}

bool ChainShape_ray_cast(b2ChainShape* self, b2RayCastOutput* output, b2RayCastInput* input, b2Transform* transform, int32 childIndex) {
    return self->RayCast(output, *input, *transform, childIndex);
}

void ChainShape_compute_aabb(b2ChainShape* self, b2AABB* aabb, const b2Transform* transform, int32 childIndex) {
    self->ComputeAABB(aabb, *transform, childIndex);
}

void ChainShape_compute_mass(b2ChainShape* self, b2MassData* massData, float density) {
    self->ComputeMass(massData, density);
}
