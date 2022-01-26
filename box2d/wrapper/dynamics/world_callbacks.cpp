#include "dynamics/world_callbacks.h"

DestructionListener* DestructionListener_new(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture, SayGoodbyeParticleGroupCB sayGoodbyeParticleGroup) {
    return new DestructionListener(sayGoodbyeJoint, sayGoodbyeFixture, sayGoodbyeParticleGroup);
}

DestructionListener DestructionListener_create(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture, SayGoodbyeParticleGroupCB sayGoodbyeParticleGroup) {
    return DestructionListener(sayGoodbyeJoint, sayGoodbyeFixture, sayGoodbyeParticleGroup);
}

ContactFilter* ContactFilter_new(ShouldCollideCB shouldCollide, ShouldCollideFixtureParticleCB shouldCollideFixtureParticle, ShouldCollideParticleParticleCB shouldCollideParticleParticle) {
    return new ContactFilter(shouldCollide, shouldCollideFixtureParticle, shouldCollideParticleParticle);
}

ContactFilter ContactFilter_create(ShouldCollideCB shouldCollide, ShouldCollideFixtureParticleCB shouldCollideFixtureParticle, ShouldCollideParticleParticleCB shouldCollideParticleParticle) {
    return ContactFilter(shouldCollide, shouldCollideFixtureParticle, shouldCollideParticleParticle);
}

// TODO I stopped here
ContactListener* ContactListener_new(BeginContactCB beginContact, EndContactCB endContact, BeginContactParticleBodyCB beginContactParticleBody, EndContactFixtureParticleCB endContactFixtureParticle, BeginContactParticleCB beginContactParticle, EndContactParticleCB endContactParticle, PreSolveCB preSolve, PostSolveCB postSolve) {
    return new ContactListener(beginContact, endContact, beginContactParticleBody, endContactFixtureParticle, beginContactParticle, endContactParticle, preSolve, postSolve);
}

ContactListener ContactListener_create(BeginContactCB beginContact, EndContactCB endContact, BeginContactParticleBodyCB beginContactParticleBody, EndContactFixtureParticleCB endContactFixtureParticle, BeginContactParticleCB beginContactParticle, EndContactParticleCB endContactParticle, PreSolveCB preSolve, PostSolveCB postSolve) {
    return ContactListener(beginContact, endContact, beginContactParticleBody, endContactFixtureParticle, beginContactParticle, endContactParticle, preSolve, postSolve);
}

QueryCallback* QueryCallback_new(ReportFixtureCB reportFixture, ReportParticleCB reportParticle, ShouldQueryParticleSystemCB shouldQueryParticleSystem) {
    return new QueryCallback(reportFixture, reportParticle, shouldQueryParticleSystem);
}

QueryCallback QueryCallback_create(ReportFixtureCB reportFixture, ReportParticleCB reportParticle, ShouldQueryParticleSystemCB shouldQueryParticleSystem) {
    return QueryCallback(reportFixture, reportParticle, shouldQueryParticleSystem);
}

RayCastCallback* RayCastCallback_new(ReportFixtureRayCastCB reportFixture, ReportParticleRayCastCB reportParticle, ShouldQueryParticleSystemRayCastCB shouldQueryParticleSystem) {
    return new RayCastCallback(reportFixture, reportParticle, shouldQueryParticleSystem);
}

RayCastCallback RayCastCallback_create(ReportFixtureRayCastCB reportFixture, ReportParticleRayCastCB reportParticle, ShouldQueryParticleSystemRayCastCB shouldQueryParticleSystem) {
    return RayCastCallback(reportFixture, reportParticle, shouldQueryParticleSystem);
}
