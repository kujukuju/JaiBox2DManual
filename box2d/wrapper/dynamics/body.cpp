#include "dynamics/body.h"

b2BodyDef* BodyDef_new() {
    return new b2BodyDef();
}

b2BodyDef BodyDef_create() {
    return b2BodyDef();
}

b2Fixture* Body_create_fixture(b2Body* self, const b2FixtureDef* def) {
    return self->CreateFixture(def);
}

b2Fixture* Body_create_fixture_density(b2Body* self, const b2Shape* shape, float density) {
    return self->CreateFixture(shape, density);
}

void Body_destroy_fixture(b2Body* self, b2Fixture* fixture) {
    self->DestroyFixture(fixture);
}

void Body_set_transform(b2Body* self, const b2Vec2* pos, float angle) {
    self->SetTransform(*pos, angle);
}

b2Transform Body_get_transform(const b2Body* self) {
    return self->GetTransform();
}

b2Vec2 Body_get_position(const b2Body* self) {
    return self->GetPosition();
}

float Body_get_angle(const b2Body* self) {
    return self->GetAngle();
}

b2Vec2 Body_get_world_center(const b2Body* self) {
    return self->GetWorldCenter();
}

b2Vec2 Body_get_local_center(const b2Body* self) {
    return self->GetLocalCenter();
}

void Body_set_linear_velocity(b2Body* self, const b2Vec2* v) {
    self->SetLinearVelocity(*v);
}

b2Vec2 Body_get_linear_velocity(const b2Body* self) {
    return self->GetLinearVelocity();
}

void Body_set_angular_velocity(b2Body* self, float omega) {
    self->SetAngularVelocity(omega);
}

float Body_get_angular_velocity(const b2Body* self) {
    return self->GetAngularVelocity();
}

void Body_apply_force(b2Body* self, const b2Vec2* force, const b2Vec2* point, bool wake) {
    self->ApplyForce(*force, *point, wake);
}

void Body_apply_force_to_center(b2Body* self, const b2Vec2* force, bool wake) {
    self->ApplyForceToCenter(*force, wake);
}

void Body_apply_torque(b2Body* self, float torque, bool wake) {
    self->ApplyTorque(torque, wake);
}

void Body_apply_linear_impulse(b2Body* self, const b2Vec2* impulse, const b2Vec2* point, bool wake) {
    self->ApplyLinearImpulse(*impulse, *point, wake);
}

void Body_apply_linear_impulse_to_center(b2Body* self, const b2Vec2* impulse, bool wake) {
    self->ApplyLinearImpulseToCenter(*impulse, wake);
}

void Body_apply_angular_impulse(b2Body* self, float impulse, bool wake) {
    self->ApplyAngularImpulse(impulse, wake);
}

float Body_get_mass(const b2Body* self) {
    return self->GetMass();
}

float Body_get_inertia(const b2Body* self) {
    return self->GetInertia();
}

b2MassData Body_get_mass_data(const b2Body* self) {
    return self->GetMassData();
}

void Body_set_mass_data(b2Body* self, const b2MassData* data) {
    self->SetMassData(data);
}

void Body_reset_mass_data(b2Body* self) {
    self->ResetMassData();
}

b2Vec2 Body_get_world_point(const b2Body* self, const b2Vec2* local) {
    return self->GetWorldPoint(*local);
}

b2Vec2 Body_get_world_vector(const b2Body* self, const b2Vec2* local) {
    return self->GetWorldVector(*local);
}

b2Vec2 Body_get_local_point(const b2Body* self, const b2Vec2* world) {
    return self->GetLocalPoint(*world);
}

b2Vec2 Body_get_local_vector(const b2Body* self, const b2Vec2* world) {
    return self->GetLocalVector(*world);
}

b2Vec2 Body_get_linear_velocity_from_world_point(const b2Body* self, const b2Vec2* point) {
    return self->GetLinearVelocityFromWorldPoint(*point);
}

b2Vec2 Body_get_linear_velocity_from_local_point(const b2Body* self, const b2Vec2* point) {
    return self->GetLinearVelocityFromLocalPoint(*point);
}

float Body_get_linear_damping(const b2Body* self) {
    return self->GetLinearDamping();
}

void Body_set_linear_damping(b2Body* self, float damping) {
    self->SetLinearDamping(damping);
}

float Body_get_angular_damping(const b2Body* self) {
    return self->GetAngularDamping();
}

void Body_set_angular_damping(b2Body* self, float damping) {
    self->SetAngularDamping(damping);
}

float Body_get_gravity_scale(const b2Body* self) {
    return self->GetGravityScale();
}

void Body_set_gravity_scale(b2Body* self, float scale) {
    self->SetGravityScale(scale);
}

void Body_set_type(b2Body* self, b2BodyType type) {
    self->SetType(static_cast<b2BodyType>(type));
}

b2BodyType Body_get_type(const b2Body* self) {
    return self->GetType();
}

void Body_set_bullet(b2Body* self, bool flag) {
    self->SetBullet(flag);
}

bool Body_is_bullet(const b2Body* self) {
    return self->IsBullet();
}

void Body_set_sleeping_allowed(b2Body* self, bool flag) {
    self->SetSleepingAllowed(flag);
}

bool Body_is_sleeping_allowed(const b2Body *self) {
    return self->IsSleepingAllowed();
}

void Body_set_awake(b2Body* self, bool flag) {
    self->SetAwake(flag);
}

bool Body_is_awake(const b2Body* self) {
    return self->IsAwake();
}

void Body_set_enabled(b2Body* self, bool flag) {
    self->SetEnabled(flag);
}

bool Body_is_enabled(const b2Body* self) {
    return self->IsEnabled();
}

void Body_set_fixed_rotation(b2Body* self, bool flag) {
    self->SetFixedRotation(flag);
}

bool Body_is_fixed_rotation(const b2Body* self) {
    return self->IsFixedRotation();
}

b2Fixture* Body_get_fixture_list(b2Body* self) {
    return self->GetFixtureList();
}

b2JointEdge* Body_get_joint_list(b2Body* self) {
    return self->GetJointList();
}

b2ContactEdge* Body_get_contact_list(b2Body* self) {
    return self->GetContactList();
}

b2Body* Body_get_next(b2Body* self) {
    return self->GetNext();
}

void* Body_get_user_data(const b2Body* self) {
    return (void*) self->GetUserData().pointer;
}

void Body_set_user_data(b2Body* self, void* data) {
    self->GetUserData().pointer = (uintptr_t) data;
}

b2World* Body_get_world(b2Body* self) {
    return self->GetWorld();
}

void Body_dump(b2Body* self) {
    self->Dump();
}
