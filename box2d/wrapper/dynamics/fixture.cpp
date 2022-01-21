#include "dynamics/fixture.h"

b2FixtureDef* FixtureDef_new() {
    return new b2FixtureDef();
}
b2FixtureDef FixtureDef_create() {
    return b2FixtureDef();
}

int32_t Fixture_get_type(const b2Fixture* self) {
    return self->GetType();
}
b2Shape* Fixture_get_shape(b2Fixture* self) {
    return self->GetShape();
}
void Fixture_set_sensor(b2Fixture* self, bool flag) {
    self->SetSensor(flag);
}
bool Fixture_is_sensor(const b2Fixture* self) {
    return self->IsSensor();
}
void Fixture_set_filter_data(b2Fixture* self, const b2Filter* filter) {
    self->SetFilterData(*filter);
}
const b2Filter* Fixture_get_filter_data(const b2Fixture* self) {
    return &self->GetFilterData();
}
void Fixture_refilter(b2Fixture* self) {
    self->Refilter();
}
b2Body* Fixture_get_body(b2Fixture* self) {
    return self->GetBody();
}
b2Fixture* Fixture_get_next(b2Fixture* self) {
    return self->GetNext();
}
void* Fixture_get_user_data(const b2Fixture* self) {
    return self->GetUserData();
}
void Fixture_set_user_data(b2Fixture* self, void* data) {
    self->SetUserData(data);
}
bool Fixture_test_point(const b2Fixture* self, const b2Vec2* p) {
    return self->TestPoint(*p);
}
bool Fixture_ray_cast(const b2Fixture* self,
                      b2RayCastOutput* output,
                      const b2RayCastInput* input,
                      int32_t child_id) {
    return self->RayCast(output, *input, child_id);
}
void Fixture_get_mass_data(const b2Fixture* self, b2MassData* data) {
    self->GetMassData(data);
}
void Fixture_set_density(b2Fixture* self, float density) {
    self->SetDensity(density);
}
float Fixture_get_density(const b2Fixture* self) {
    return self->GetDensity();
}
float Fixture_get_friction(const b2Fixture* self) {
    return self->GetFriction();
}
void Fixture_set_friction(b2Fixture* self, float friction) {
    self->SetFriction(friction);
}
float Fixture_get_restitution(const b2Fixture* self) {
    return self->GetRestitution();
}
void Fixture_set_restitution(b2Fixture* self, float restitution) {
    self->SetRestitution(restitution);
}
const b2AABB* Fixture_get_aabb(const b2Fixture* self, int32_t child_id) {
    return &self->GetAABB(child_id);
}

void Fixture_dump(b2Fixture* self, int32_t body_id) {
    self->Dump(body_id);
}
