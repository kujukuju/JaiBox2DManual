#include "particle/particle_group.h"

b2ParticleGroupDef* ParticleGroupDef_new() {
    return new b2ParticleGroupDef();
}

b2ParticleGroupDef ParticleGroupDef_create() {
    return b2ParticleGroupDef();
}

b2ParticleGroup* ParticleGroup_get_next(b2ParticleGroup* self) {
    return self->GetNext();
}

b2ParticleSystem* ParticleGroup_get_particle_system(b2ParticleGroup* self) {
    return self->GetParticleSystem();
}

int32 ParticleGroup_get_particle_count(b2ParticleGroup* self) {
    return self->GetParticleCount();
}

int32 ParticleGroup_get_buffer_index(b2ParticleGroup* self) {
    return self->GetBufferIndex();
}

bool ParticleGroup_contains_particle(b2ParticleGroup* self, int32 index) {
    return self->ContainsParticle(index);
}

uint32 ParticleGroup_get_all_particle_flags(b2ParticleGroup* self) {
    return self->GetAllParticleFlags();
}

uint32 ParticleGroup_get_group_flags(b2ParticleGroup* self) {
    return self->GetGroupFlags();
}

void ParticleGroup_set_group_flags(b2ParticleGroup* self, uint32 flags) {
    self->SetGroupFlags(flags);
}

float32 ParticleGroup_get_mass(b2ParticleGroup* self) {
    return self->GetMass();
}

float32 ParticleGroup_get_inertia(b2ParticleGroup* self) {
    return self->GetInertia();
}

b2Vec2 ParticleGroup_get_center(b2ParticleGroup* self) {
    return self->GetCenter();
}

b2Vec2 ParticleGroup_get_linear_velocity(b2ParticleGroup* self) {
    return self->GetLinearVelocity();
}

float32 ParticleGroup_get_angular_velocity(b2ParticleGroup* self) {
    return self->GetAngularVelocity();
}

b2Transform ParticleGroup_get_transform(b2ParticleGroup* self) {
    return self->GetTransform();
}

b2Vec2 ParticleGroup_get_position(b2ParticleGroup* self) {
    return self->GetPosition();
}

float32 ParticleGroup_get_angle(b2ParticleGroup* self) {
    return self->GetAngle();
}

b2Vec2 ParticleGroup_get_linear_velocity_from_world_point(b2ParticleGroup* self, const b2Vec2* worldPoint) {
    return self->GetLinearVelocityFromWorldPoint(*worldPoint);
}

void* ParticleGroup_get_user_data(b2ParticleGroup* self) {
    return self->GetUserData();
}

void ParticleGroup_set_user_data(b2ParticleGroup* self, void* data) {
    self->SetUserData(data);
}

void ParticleGroup_apply_force(b2ParticleGroup* self, const b2Vec2* force) {
    self->ApplyForce(*force);
}

void ParticleGroup_apply_linear_impulse(b2ParticleGroup* self, const b2Vec2* impulse) {
    self->ApplyLinearImpulse(*impulse);
}

void ParticleGroup_destroy_particles(b2ParticleGroup* self, bool callDestructionListener) {
    self->DestroyParticles(callDestructionListener);
}
