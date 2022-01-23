#include "dynamics/world_callbacks.h"

DestructionListener* DestructionListener_new(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture) {
    return new DestructionListener(sayGoodbyeJoint, sayGoodbyeFixture);
}

DestructionListener DestructionListener_create(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture) {
    return DestructionListener(sayGoodbyeJoint, sayGoodbyeFixture);
}

ContactFilter* ContactFilter_new(ShouldCollideCB shouldCollide) {
    return new ContactFilter(shouldCollide);
}

ContactFilter ContactFilter_create(ShouldCollideCB shouldCollide) {
    return ContactFilter(shouldCollide);
}

ContactListener* ContactListener_new(BeginContactCB beginContact, EndContactCB endContact, PreSolveCB preSolve, PostSolveCB postSolve) {
    return new ContactListener(beginContact, endContact, preSolve, postSolve);
}

ContactListener ContactListener_create(BeginContactCB beginContact, EndContactCB endContact, PreSolveCB preSolve, PostSolveCB postSolve) {
    return ContactListener(beginContact, endContact, preSolve, postSolve);
}

QueryCallback* QueryCallback_new(ReportFixtureQueryCB reportFixture) {
    return new QueryCallback(reportFixture);
}

QueryCallback QueryCallback_create(ReportFixtureQueryCB reportFixture) {
    return QueryCallback(reportFixture);
}

RayCastCallback* RayCastCallback_new(ReportFixtureRayCastCB reportFixture) {
    return new RayCastCallback(reportFixture);
}

RayCastCallback RayCastCallback_create(ReportFixtureRayCastCB reportFixture) {
    return RayCastCallback(reportFixture);
}
