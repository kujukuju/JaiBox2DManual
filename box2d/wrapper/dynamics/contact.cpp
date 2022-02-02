#include "dynamics/contact.h"

float b2_mix_friction(float friction1, float friction2) {
    return b2MixFriction(friction1, friction2);
}

float b2_mix_restitution(float restitution1, float restitution2) {
    return b2MixRestitution(restitution1, restitution2);
}

float b2_mix_restitution_threshold(float threshold1, float threshold2) {
    return b2MixRestitutionThreshold(threshold1, threshold2);
}

b2Manifold* Contact_get_manifold(b2Contact* self) {
    return self->GetManifold();
}

void Contact_get_world_manifold(b2Contact* self, b2WorldManifold* worldManifold) {
    self->GetWorldManifold(worldManifold);
}

bool Contact_is_touching(b2Contact* self) {
    return self->IsTouching();
}

void Contact_set_enabled(b2Contact* self, bool flag) {
    self->SetEnabled(flag);
}

bool Contact_is_enabled(b2Contact* self) {
    return self->IsEnabled();
}

b2Contact* Contact_get_next(b2Contact* self) {
    return self->GetNext();
}

b2Fixture* Contact_get_fixture_a(b2Contact* self) {
    return self->GetFixtureA();
}

int32 Contact_get_child_index_a(b2Contact* self) {
    return self->GetChildIndexA();
}

b2Fixture* Contact_get_fixture_b(b2Contact* self) {
    return self->GetFixtureB();
}

int32 Contact_get_child_index_b(b2Contact* self) {
    return self->GetChildIndexB();
}

void Contact_set_friction(b2Contact* self, float friction) {
    self->SetFriction(friction);
}

float Contact_get_friction(b2Contact* self) {
    return self->GetFriction();
}

void Contact_reset_friction(b2Contact* self) {
    self->ResetFriction();
}

void Contact_set_restitution(b2Contact* self, float restitution) {
    self->SetRestitution(restitution);
}

float Contact_get_restitution(b2Contact* self) {
    return self->GetRestitution();
}

void Contact_reset_restitution(b2Contact* self) {
    self->ResetRestitution();
}

void Contact_set_restitution_threshold(b2Contact* self, float threshold) {
    self->SetRestitutionThreshold(threshold);
}

float Contact_get_restitution_threshold(b2Contact* self) {
    return self->GetRestitutionThreshold();
}

void Contact_reset_restitution_threshold(b2Contact* self) {
    self->ResetRestitutionThreshold();
}

void Contact_set_tangent_speed(b2Contact* self, float speed) {
    self->SetTangentSpeed(speed);
}

float Contact_get_tangent_speed(b2Contact* self) {
    return self->GetTangentSpeed();
}

void Contact_evaluate(b2Contact* self, b2Manifold* manifold, const b2Transform* xfA, const b2Transform* xfB) {
    self->Evaluate(manifold, *xfA, *xfB);
}
