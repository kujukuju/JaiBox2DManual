#include "dynamics/joints/rope_joint.h"

b2RopeTuning* RopeTuning_new() {
    return new b2RopeTuning();
}

b2RopeTuning RopeTurning_create() {
    return b2RopeTuning();
}

b2RopeDef* RopeDef_new() {
    return new b2RopeDef();
}

b2RopeDef RopeDef_create() {
    return b2RopeDef();
}

b2Rope* Rope_new() {
    return new b2Rope();
}

b2Rope Rope_create() {
    return b2Rope();
}

void Rope_initialize(b2Rope* self, const b2RopeDef* def) {
    self->Create(*def);
}

void Rope_set_tuning(b2Rope* self, const b2RopeTuning* tuning) {
    self->SetTuning(*tuning);
}

void Rope_step(b2Rope* self, float timeStep, int32_t iterations, const b2Vec2* position) {
    self->Step(timeStep, iterations, *position);
}

void Rope_reset(b2Rope* self, const b2Vec2* position) {
    self->Reset(*position);
}

void Rope_draw(b2Rope* self, b2Draw* draw) {
    self->Draw(draw);
}
