#include "dynamics/joints/joint.h"

b2JointDef* JointDef_new() {
    return new b2JointDef();
}

b2JointDef JointDef_create() {
    return b2JointDef();
}

void linear_stiffness(float* stiffness, float* damping, float frequencyHertz, float dampingRatio, const b2Body* bodyA, const b2Body* bodyB) {
    b2LinearStiffness(*stiffness, *damping, frequencyHertz, dampingRatio, bodyA, bodyB);
}

void angular_stiffness(float* stiffness, float* damping, float frequencyHertz, float dampingRatio, const b2Body* bodyA, const b2Body* bodyB) {
    b2AngularStiffness(*stiffness, *damping, frequencyHertz, dampingRatio, bodyA, bodyB);
}

b2JointType Joint_get_type(b2Joint* self) {
    return self->GetType();
}

b2Body* Joint_get_body_a(b2Joint* self) {
    return self->GetBodyA();
}

b2Body* Joint_get_body_b(b2Joint* self) {
    return self->GetBodyB();
}

b2Vec2 Joint_get_anchor_a(b2Joint* self) {
    return self->GetAnchorA();
}

b2Vec2 Joint_get_anchor_b(b2Joint* self) {
    return self->GetAnchorB();
}

b2Vec2 Joint_get_reaction_force(b2Joint* self, float inv_dt) {
    return self->GetReactionForce(inv_dt);
}

float Joint_get_reaction_torque(b2Joint* self, float inv_dt) {
    return self->GetReactionTorque(inv_dt);
}

b2Joint* Joint_get_next(b2Joint* self) {
    return self->GetNext();
}

void* Joint_get_user_data(b2Joint* self) {
    return (void*) self->GetUserData().pointer;
}

void Joint_set_user_data(b2Joint* self, void* data) {
    self->GetUserData().pointer = (uintptr_t) data;
}

bool Joint_is_enabled(b2Joint* self) {
    return self->IsEnabled();
}

bool Joint_get_collide_connected(b2Joint* self) {
    return self->GetCollideConnected();
}

void Joint_dump(b2Joint* self) {
    self->Dump();
}

void Joint_shift_origin(b2Joint* self, const b2Vec2* newOrigin) {
    self->ShiftOrigin(*newOrigin);
}

void Joint_draw(b2Joint* self, b2Draw* draw) {
    self->Draw(draw);
}
