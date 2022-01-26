#include "particle/particle_system.h"

void ParticleContact_set_indices(b2ParticleContact* self, int32 a, int32 b) {
    self->SetIndices(a, b);
}

void ParticleContact_set_weight(b2ParticleContact* self, float32 w) {
    self->SetWeight(w);
}

void ParticleContact_set_normal(b2ParticleContact* self, const b2Vec2* n) {
    self->SetNormal(*n);
}

void ParticleContact_set_flags(b2ParticleContact* self, uint32 f) {
    self->SetFlags(f);
}

int32 ParticleContact_get_index_a(b2ParticleContact* self) {
    return self->GetIndexA();
}

int32 ParticleContact_get_index_b(b2ParticleContact* self) {
    return self->GetIndexB();
}

float32 ParticleContact_get_weight(b2ParticleContact* self) {
    return self->GetWeight();
}

b2Vec2 ParticleContact_get_normal(b2ParticleContact* self) {
    return self->GetNormal();
}

uint32 ParticleContact_get_flags(b2ParticleContact* self) {
    return self->GetFlags();
}

b2ParticleSystemDef* ParticleSystemDef_new() {
    return new b2ParticleSystemDef();
}

b2ParticleSystemDef ParticleSystemDef_create() {
    return b2ParticleSystemDef();
}

// b2ParticleSystem* ParticleSystem_new(b2ParticleSystemDef def) {
//     return new b2ParticleSystem(def);
// }

// b2ParticleSystem ParticleSystem_create(b2ParticleSystemDef def) {
//     return b2ParticleSystem(def);
// }

int32 ParticleSystem_create_particle(b2ParticleSystem* self, const b2ParticleDef* def) {
    return self->CreateParticle(*def);
}

const b2ParticleHandle* ParticleSystem_get_particle_handle_from_index(b2ParticleSystem* self, const int32 index) {
    return self->GetParticleHandleFromIndex(index);
}

void ParticleSystem_destroy_particle(b2ParticleSystem* self, int32 index, bool callDestructionListener) {
    self->DestroyParticle(index, callDestructionListener);
}

void ParticleSystem_destroy_oldest_particle(b2ParticleSystem* self, const int32 index, const bool callDestructionListener) {
    self->DestroyOldestParticle(index, callDestructionListener);
}

int32 ParticleSystem_destroy_particles_in_shape(b2ParticleSystem* self, const b2Shape* shape, const b2Transform* xf, bool callDestructionListener) {
    return self->DestroyParticlesInShape(*shape, *xf, callDestructionListener);
}

b2ParticleGroup* ParticleSystem_create_particle_group(b2ParticleSystem* self, const b2ParticleGroupDef* def) {
    return self->CreateParticleGroup(*def);
}

void ParticleSystem_join_particle_groups(b2ParticleSystem* self, b2ParticleGroup* groupA, b2ParticleGroup* groupB) {
    self->JoinParticleGroups(groupA, groupB);
}

void ParticleSystem_split_particle_group(b2ParticleSystem* self, b2ParticleGroup* group) {
    self->SplitParticleGroup(group);
}

b2ParticleGroup* ParticleSystem_get_particle_group_list(b2ParticleSystem* self) {
    return self->GetParticleGroupList();
}

int32 ParticleSystem_get_particle_group_count(b2ParticleSystem* self) {
    return self->GetParticleGroupCount();
}

int32 ParticleSystem_get_particle_count(b2ParticleSystem* self) {
    return self->GetParticleCount();
}

int32 ParticleSystem_get_max_particle_count(b2ParticleSystem* self) {
    return self->GetMaxParticleCount();
}

void ParticleSystem_set_max_particle_count(b2ParticleSystem* self, int32 count) {
    self->SetMaxParticleCount(count);
}

uint32 ParticleSystem_get_all_particle_flags(b2ParticleSystem* self) {
    return self->GetAllParticleFlags();
}

uint32 ParticleSystem_get_all_group_flags(b2ParticleSystem* self) {
    return self->GetAllGroupFlags();
}

void ParticleSystem_set_paused(b2ParticleSystem* self, bool paused) {
    self->SetPaused(paused);
}

bool ParticleSystem_get_paused(b2ParticleSystem* self) {
    return self->GetPaused();
}

void ParticleSystem_set_density(b2ParticleSystem* self, float32 density) {
    self->SetDensity(density);
}

float32 ParticleSystem_get_density(b2ParticleSystem* self) {
    return self->GetDensity();
}

void ParticleSystem_set_gravity_scale(b2ParticleSystem* self, float32 gravityScale) {
    self->SetGravityScale(gravityScale);
}

float32 ParticleSystem_get_gravity_scale(b2ParticleSystem* self) {
    return self->GetGravityScale();
}

void ParticleSystem_set_damping(b2ParticleSystem* self, float32 damping) {
    self->SetDamping(damping);
}

float32 ParticleSystem_get_damping(b2ParticleSystem* self) {
    return self->GetDamping();
}

void ParticleSystem_set_static_pressure_iterations(b2ParticleSystem* self, int32 iterations) {
    self->SetStaticPressureIterations(iterations);
}

int32 ParticleSystem_get_static_pressure_iterations(b2ParticleSystem* self) {
    return self->GetStaticPressureIterations();
}

void ParticleSystem_set_radius(b2ParticleSystem* self, float32 radius) {
    self->SetRadius(radius);
}

float32 ParticleSystem_get_radius(b2ParticleSystem* self) {
    return self->GetRadius();
}

b2Vec2* ParticleSystem_get_position_buffer(b2ParticleSystem* self) {
    return self->GetPositionBuffer();
}

b2Vec2* ParticleSystem_get_velocity_buffer(b2ParticleSystem* self) {
    return self->GetVelocityBuffer();
}

b2ParticleColor* ParticleSystem_get_color_buffer(b2ParticleSystem* self) {
    return self->GetColorBuffer();
}

// b2ParticleGroup* const* ParticleSystem_get_group_buffer(b2ParticleSystem* self) {
b2ParticleGroup** ParticleSystem_get_group_buffer(b2ParticleSystem* self) {
    return const_cast<b2ParticleGroup**>(self->GetGroupBuffer());
}

float32* ParticleSystem_get_weight_buffer(b2ParticleSystem* self) {
    return self->GetWeightBuffer();
}

void** ParticleSystem_get_user_data_buffer(b2ParticleSystem* self) {
    return self->GetUserDataBuffer();
}

const uint32* ParticleSystem_get_flags_buffer(b2ParticleSystem* self) {
    return self->GetFlagsBuffer();
}

void ParticleSystem_set_particle_flags(b2ParticleSystem* self, int32 index, uint32 flags) {
    self->SetParticleFlags(index, flags);
}

uint32 ParticleSystem_get_particle_flags(b2ParticleSystem* self, const int32 index) {
    return self->GetParticleFlags(index);
}

void ParticleSystem_set_flags_buffer(b2ParticleSystem* self, uint32* buffer, int32 capacity) {
    self->SetFlagsBuffer(buffer, capacity);
}

void ParticleSystem_set_position_buffer(b2ParticleSystem* self, b2Vec2* buffer, int32 capacity) {
    self->SetPositionBuffer(buffer, capacity);
}

void ParticleSystem_set_velocity_buffer(b2ParticleSystem* self, b2Vec2* buffer, int32 capacity) {
    self->SetVelocityBuffer(buffer, capacity);
}

void ParticleSystem_set_color_buffer(b2ParticleSystem* self, b2ParticleColor* buffer, int32 capacity) {
    self->SetColorBuffer(buffer, capacity);
}

void ParticleSystem_set_user_data_buffer(b2ParticleSystem* self, void** buffer, int32 capacity) {
    self->SetUserDataBuffer(buffer, capacity);
}

const b2ParticleContact* ParticleSystem_get_contacts(b2ParticleSystem* self) {
    return self->GetContacts();
}

int32 ParticleSystem_get_contact_count(b2ParticleSystem* self) {
    return self->GetContactCount();
}

const b2ParticleBodyContact* ParticleSystem_get_body_contacts(b2ParticleSystem* self) {
    return self->GetBodyContacts();
}

int32 ParticleSystem_get_body_contact_count(b2ParticleSystem* self) {
    return self->GetBodyContactCount();
}

const b2ParticlePair* ParticleSystem_get_pairs(b2ParticleSystem* self) {
    return self->GetPairs();
}

int32 ParticleSystem_get_pair_count(b2ParticleSystem* self) {
    return self->GetPairCount();
}

const b2ParticleTriad* ParticleSystem_get_triads(b2ParticleSystem* self) {
    return self->GetTriads();
}

int32 ParticleSystem_get_triad_count(b2ParticleSystem* self) {
    return self->GetTriadCount();
}

void ParticleSystem_set_stuck_threshold(b2ParticleSystem* self, int32 iterations) {
    self->SetStuckThreshold(iterations);
}

const int32* ParticleSystem_get_stuck_candidates(b2ParticleSystem* self) {
    return self->GetStuckCandidates();
}

int32 ParticleSystem_get_stuck_candidate_count(b2ParticleSystem* self) {
    return self->GetStuckCandidateCount();
}

float32 ParticleSystem_compute_collision_energy(b2ParticleSystem* self) {
    return self->ComputeCollisionEnergy();
}

void ParticleSystem_set_strict_contact_check(b2ParticleSystem* self, bool enabled) {
    self->SetStrictContactCheck(enabled);
}

bool ParticleSystem_get_strict_contact_check(b2ParticleSystem* self) {
    return self->GetStrictContactCheck();
}

void ParticleSystem_set_particle_lifetime(b2ParticleSystem* self, const int32 index, const float32 lifetime) {
    self->SetParticleLifetime(index, lifetime);
}

float32 ParticleSystem_get_particle_lifetime(b2ParticleSystem* self, const int32 index) {
    return self->GetParticleLifetime(index);
}

void ParticleSystem_set_destruction_by_age(b2ParticleSystem* self, const bool enable) {
    self->SetDestructionByAge(enable);
}

bool ParticleSystem_get_destruction_by_age(b2ParticleSystem* self) {
    return self->GetDestructionByAge();
}

const int32* ParticleSystem_get_expiration_time_buffer(b2ParticleSystem* self) {
    return self->GetExpirationTimeBuffer();
}

float32 ParticleSystem_expiration_time_to_lifetime(b2ParticleSystem* self, const int32 expirationTime) {
    return self->ExpirationTimeToLifetime(expirationTime);
}

const int32* ParticleSystem_get_index_by_expiration_time_buffer(b2ParticleSystem* self) {
    return self->GetIndexByExpirationTimeBuffer();
}

void ParticleSystem_particle_apply_linear_impulse(b2ParticleSystem* self, int32 index, const b2Vec2* impulse) {
    self->ParticleApplyLinearImpulse(index, *impulse);
}

void ParticleSystem_apply_linear_impulse(b2ParticleSystem* self, int32 firstIndex, int32 lastIndex, const b2Vec2* impulse) {
    self->ApplyLinearImpulse(firstIndex, lastIndex, *impulse);
}

void ParticleSystem_particle_apply_force(b2ParticleSystem* self, int32 index, const b2Vec2* force) {
    self->ParticleApplyForce(index, *force);
}

void ParticleSystem_apply_force(b2ParticleSystem* self, int32 firstIndex, int32 lastIndex, const b2Vec2* force) {
    self->ApplyForce(firstIndex, lastIndex, *force);
}

b2ParticleSystem* ParticleSystem_get_next(b2ParticleSystem* self) {
    return self->GetNext();
}

void ParticleSystem_query_aabb(b2ParticleSystem* self, b2QueryCallback* callback, const b2AABB* aabb) {
    self->QueryAABB(callback, *aabb);
}

void ParticleSystem_query_shape_aabb(b2ParticleSystem* self, b2QueryCallback* callback, const b2Shape* shape, const b2Transform* xf) {
    self->QueryShapeAABB(callback, *shape, *xf);
}

void ParticleSystem_ray_cast(b2ParticleSystem* self, b2RayCastCallback* callback, const b2Vec2* point1, const b2Vec2* point2) {
    self->RayCast(callback, *point1, *point2);
}

void ParticleSystem_compute_aabb(b2ParticleSystem* self, b2AABB* const aabb) {
    self->ComputeAABB(aabb);
}
