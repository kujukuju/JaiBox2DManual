#include "collision/shapes/circle_shape.h"

b2CircleShape* CircleShape_new() {
    return new b2CircleShape();
}

b2CircleShape CircleShape_create() {
    return b2CircleShape();
}

bool CircleShape_test_point(b2CircleShape* self, b2Transform* transform, b2Vec2* p) {
    return self->TestPoint(*transform, *p);
}

void CircleShape_compute_distance(b2CircleShape* self, const b2Transform* xf, const b2Vec2* p, float32* distance, b2Vec2* normal, int32 childIndex) {
    self->ComputeDistance(*xf, *p, distance, normal, childIndex);
}

bool CircleShape_ray_cast(b2CircleShape* self, b2RayCastOutput* output, const b2RayCastInput* input, const b2Transform* transform, int32 childIndex) {
    return self->RayCast(output, *input, *transform, childIndex);
}

void CircleShape_compute_aabb(b2CircleShape* self, b2AABB* aabb, const b2Transform* transform, int32 childIndex) {
    self->ComputeAABB(aabb, *transform, childIndex);
}

void CircleShape_compute_mass(b2CircleShape* self, b2MassData* massData, float density) {
    self->ComputeMass(massData, density);
}

void CircleShape_set_position(b2CircleShape* self, float32 x, float32 y) {
    self->SetPosition(x, y);
}

float32 CircleShape_get_position_x(b2CircleShape* self) {
    return self->GetPositionX();
}

float32 CircleShape_get_position_y(b2CircleShape* self) {
    return self->GetPositionY();
}