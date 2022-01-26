#include "dynamics/world.h"

b2World* World_new(const b2Vec2 gravity) {
    return new b2World(gravity);
}

b2World World_create(const b2Vec2 gravity) {
    return b2World(gravity);
}

void World_set_destruction_listener(b2World* self, b2DestructionListener* listener) {
    self->SetDestructionListener(listener);
}

void World_set_contact_filter(b2World* self, b2ContactFilter* filter) {
    self->SetContactFilter(filter);
}

void World_set_contact_listener(b2World* self, b2ContactListener* listener) {
    self->SetContactListener(listener);
}

void World_set_debug_draw(b2World* self, b2Draw* debugDraw) {
    self->SetDebugDraw(debugDraw);
}

b2Body* World_create_body(b2World* self, const b2BodyDef* def) {
    return self->CreateBody(def);
}

void World_destroy_body(b2World* self, b2Body* body) {
    self->DestroyBody(body);
}

b2Joint* World_create_joint(b2World* self, const b2JointDef* def) {
    return self->CreateJoint(def);
}

void World_destroy_joint(b2World* self, b2Joint* joint) {
    self->DestroyJoint(joint);
}

b2ParticleSystem* World_create_particle_system(b2World* self, const b2ParticleSystemDef* def) {
    return self->CreateParticleSystem(def);
}

void World_destroy_particle_system(b2World* self, b2ParticleSystem* p) {
    self->DestroyParticleSystem(p);
}

void World_step(b2World* self, float timeStep, int32_t velocityIterations, int32_t positionIterations, int32_t particleIterations) {
    self->Step(timeStep, velocityIterations, positionIterations, particleIterations);
}

int World_calculate_reasonable_particle_iterations(b2World* self, float32 timeStep) {
    return self->CalculateReasonableParticleIterations(timeStep);
}

void World_clear_forces(b2World* self) {
    self->ClearForces();
}

void World_debug_draw(b2World* self) {
    self->DebugDraw();
}

void World_query_aabb(b2World* self, b2QueryCallback* callback, const b2AABB* aabb) {
    self->QueryAABB(callback, *aabb);
}

void World_query_shape_aabb(b2World* self, b2QueryCallback* callback, const b2Shape* shape, const b2Transform* xf) {
    self->QueryShapeAABB(callback, *shape, *xf);
}

void World_ray_cast(b2World* self, b2RayCastCallback* callback, const b2Vec2* p1, const b2Vec2* p2) {
    self->RayCast(callback, *p1, *p2);
}

b2Body* World_get_body_list(b2World* self) {
    return self->GetBodyList();
}

b2Joint* World_get_joint_list(b2World* self) {
    return self->GetJointList();
}

b2ParticleSystem* World_get_particle_system_list(b2World* self) {
    return self->GetParticleSystemList();
}

b2Contact* World_get_contact_list(b2World* self) {
    return self->GetContactList();
}

void World_set_allow_sleeping(b2World* self, bool flag) {
    self->SetAllowSleeping(flag);
}

bool World_get_allow_sleeping(b2World* self) {
    return self->GetAllowSleeping();
}

void World_set_warm_starting(b2World* self, bool flag) {
    self->SetWarmStarting(flag);
}

bool World_get_warm_starting(b2World* self) {
    return self->GetWarmStarting();
}

void World_set_continuous_physics(b2World* self, bool flag) {
    self->SetContinuousPhysics(flag);
}

bool World_get_continuous_physics(b2World* self) {
    return self->GetContinuousPhysics();
}

void World_set_sub_stepping(b2World* self, bool flag) {
    self->SetSubStepping(flag);
}

bool World_get_sub_stepping(b2World* self) {
    return self->GetSubStepping();
}

int32_t World_get_proxy_count(b2World* self) {
    return self->GetProxyCount();
}

int32_t World_get_body_count(b2World* self) {
    return self->GetBodyCount();
}

int32_t World_get_joint_count(b2World* self) {
    return self->GetJointCount();
}

int32_t World_get_contact_count(b2World* self) {
    return self->GetContactCount();
}

int32_t World_get_tree_height(b2World* self) {
    return self->GetTreeHeight();
}

int32_t World_get_tree_balance(b2World* self) {
    return self->GetTreeBalance();
}

float World_get_tree_quality(b2World* self) {
    return self->GetTreeQuality();
}

void World_set_gravity(b2World* self, const b2Vec2* gravity) {
    self->SetGravity(*gravity);
}

b2Vec2 World_get_gravity(b2World* self) {
    return self->GetGravity();
}

bool World_is_locked(b2World* self) {
    return self->IsLocked();
}

void World_set_auto_clear_forces(b2World* self, bool flag) {
    self->SetAutoClearForces(flag);
}

bool World_get_auto_clear_forces(b2World* self) {
    return self->GetAutoClearForces();
}

void World_shift_origin(b2World* self, const b2Vec2* origin) {
    self->ShiftOrigin(*origin);
}

b2ContactManager World_get_contact_manager(b2World* self) {
    return self->GetContactManager();
}

b2Profile World_get_profile(b2World* self) {
    return self->GetProfile();
}

void World_dump(b2World* self) {
    self->Dump();
}
