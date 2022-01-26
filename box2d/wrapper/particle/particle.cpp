#include "particle/particle.h"

b2ParticleDef* ParticleDef_new() {
    return new b2ParticleDef();
}

b2ParticleDef ParticleDef_create() {
    return b2ParticleDef();
}

int32 b2_calculate_particle_iterations(float32 gravity, float32 radius, float32 timeStep) {
    return b2CalculateParticleIterations(gravity, radius, timeStep);
}
