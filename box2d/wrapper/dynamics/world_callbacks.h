typedef void (*SayGoodbyeJointCB)(b2Joint*);
typedef void (*SayGoodbyeFixtureCB)(b2Fixture*);
typedef void (*SayGoodbyeParticleGroupCB)(b2ParticleGroup*);

struct DestructionListener : public b2DestructionListener {
    DestructionListener(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture, SayGoodbyeParticleGroupCB sayGoodbyeParticleGroup)
        : m_sayGoodbyeJoint(*sayGoodbyeJoint),
          m_sayGoodbyeFixture(*sayGoodbyeFixture),
          m_sayGoodbyeParticleGroup(*sayGoodbyeParticleGroup) {}

    ~DestructionListener() {}

    void SayGoodbye(b2Joint* joint) override {
        m_sayGoodbyeJoint(joint);
    }

    void SayGoodbye(b2Fixture* fixture) override {
        m_sayGoodbyeFixture(fixture);
    }

    void SayGoodbye(b2ParticleGroup* group) override {
        m_sayGoodbyeParticleGroup(group);
    }

    SayGoodbyeJointCB m_sayGoodbyeJoint;
    SayGoodbyeFixtureCB m_sayGoodbyeFixture;
    SayGoodbyeParticleGroupCB m_sayGoodbyeParticleGroup;
};

EXPORT DestructionListener* DestructionListener_new(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture, SayGoodbyeParticleGroupCB sayGoodbyeParticleGroup);
EXPORT DestructionListener DestructionListener_create(SayGoodbyeJointCB sayGoodbyeJoint, SayGoodbyeFixtureCB sayGoodbyeFixture, SayGoodbyeParticleGroupCB sayGoodbyeParticleGroup);

typedef bool (*ShouldCollideCB)(b2Fixture*, b2Fixture*);
typedef bool (*ShouldCollideFixtureParticleCB)(b2Fixture*, b2ParticleSystem*, int32);
typedef bool (*ShouldCollideParticleParticleCB)(b2ParticleSystem*, int32, int32);

struct ContactFilter : public b2ContactFilter {
    ContactFilter(ShouldCollideCB shouldCollide, ShouldCollideFixtureParticleCB shouldCollideFixtureParticle, ShouldCollideParticleParticleCB shouldCollideParticleParticle)
        : m_shouldCollide(*shouldCollide),
          m_shouldCollideFixtureParticle(*shouldCollideFixtureParticle),
          m_shouldCollideParticleParticle(*shouldCollideParticleParticle) {}

    ~ContactFilter() {}

    bool ShouldCollide(b2Fixture* fixtureA, b2Fixture* fixtureB) override {
        return m_shouldCollide(fixtureA, fixtureB);
    }

    bool ShouldCollide(b2Fixture* fixture, b2ParticleSystem* particleSystem, int32 particleIndex) override {
        return m_shouldCollideFixtureParticle(fixture, particleSystem, particleIndex);
    }

    bool ShouldCollide(b2ParticleSystem* particleSystem, int32 particleIndexA, int32 particleIndexB) {
        return m_shouldCollideParticleParticle(particleSystem, particleIndexA, particleIndexB);
    }

    ShouldCollideCB m_shouldCollide;
    ShouldCollideFixtureParticleCB m_shouldCollideFixtureParticle;
    ShouldCollideParticleParticleCB m_shouldCollideParticleParticle;
};

EXPORT ContactFilter* ContactFilter_new(ShouldCollideCB shouldCollide, ShouldCollideFixtureParticleCB shouldCollideFixtureParticle, ShouldCollideParticleParticleCB shouldCollideParticleParticle);
EXPORT ContactFilter ContactFilter_create(ShouldCollideCB shouldCollide, ShouldCollideFixtureParticleCB shouldCollideFixtureParticle, ShouldCollideParticleParticleCB shouldCollideParticleParticle);

typedef void (*BeginContactCB)(b2Contact*);
typedef void (*EndContactCB)(b2Contact*);
typedef void (*BeginContactParticleBodyCB)(b2ParticleSystem*, b2ParticleBodyContact*);
typedef void (*EndContactFixtureParticleCB)(b2Fixture*, b2ParticleSystem*, int32);
typedef void (*BeginContactParticleCB)(b2ParticleSystem*, b2ParticleContact*);
typedef void (*EndContactParticleCB)(b2ParticleSystem*, int32, int32);
typedef void (*PreSolveCB)(b2Contact*, const b2Manifold*);
typedef void (*PostSolveCB)(b2Contact*, const b2ContactImpulse*);

struct ContactListener : public b2ContactListener {
    ContactListener(
        BeginContactCB beginContact,
        EndContactCB endContact,
        BeginContactParticleBodyCB beginContactParticleBody,
        EndContactFixtureParticleCB endContactFixtureParticle,
        BeginContactParticleCB beginContactParticle,
        EndContactParticleCB endContactParticle,
        PreSolveCB preSolve,
        PostSolveCB postSolve)
        : m_beginContact(*beginContact),
          m_endContact(*endContact),
          m_beginContactParticleBody(*beginContactParticleBody),
          m_endContactFixtureParticle(*endContactFixtureParticle),
          m_beginContactParticle(*beginContactParticle),
          m_endContactParticle(*endContactParticle),
          m_preSolve(*preSolve),
          m_postSolve(*postSolve) {}

    ~ContactListener() {}

    void BeginContact(b2Contact* contact) override {
        m_beginContact(contact);
    }

    void EndContact(b2Contact* contact) override {
        m_endContact(contact);
    }

    void BeginContact(b2ParticleSystem* particleSystem, b2ParticleBodyContact* particleBodyContact) override {
        m_beginContactParticleBody(particleSystem, particleBodyContact);
    }

    void EndContact(b2Fixture* fixture, b2ParticleSystem* particleSystem, int32 index) override {
        m_endContactFixtureParticle(fixture, particleSystem, index);
    }

    void BeginContact(b2ParticleSystem* particleSystem, b2ParticleContact* particleContact) override {
        m_beginContactParticle(particleSystem, particleContact);
    }

    void EndContact(b2ParticleSystem* particleSystem, int32 indexA, int32 indexB) override {
        m_endContactParticle(particleSystem, indexA, indexB);
    }

    void PreSolve(b2Contact* contact, const b2Manifold* oldManifold) override {
        m_preSolve(contact, oldManifold);
    }

    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) override {
        m_postSolve(contact, impulse);
    }

    BeginContactCB m_beginContact;
    EndContactCB m_endContact;
    BeginContactParticleBodyCB m_beginContactParticleBody;
    EndContactFixtureParticleCB m_endContactFixtureParticle;
    BeginContactParticleCB m_beginContactParticle;
    EndContactParticleCB m_endContactParticle;
    PreSolveCB m_preSolve;
    PostSolveCB m_postSolve;
};

EXPORT ContactListener* ContactListener_new(BeginContactCB beginContact, EndContactCB endContact, BeginContactParticleBodyCB beginContactParticleBody, EndContactFixtureParticleCB endContactFixtureParticle, BeginContactParticleCB beginContactParticle, EndContactParticleCB endContactParticle, PreSolveCB preSolve, PostSolveCB postSolve);
EXPORT ContactListener ContactListener_create(BeginContactCB beginContact, EndContactCB endContact, BeginContactParticleBodyCB beginContactParticleBody, EndContactFixtureParticleCB endContactFixtureParticle, BeginContactParticleCB beginContactParticle, EndContactParticleCB endContactParticle, PreSolveCB preSolve, PostSolveCB postSolve);

typedef bool (*ReportFixtureCB)(b2Fixture*);
typedef bool (*ReportParticleCB)(const b2ParticleSystem*, int32);
typedef bool (*ShouldQueryParticleSystemCB)(const b2ParticleSystem*);

struct QueryCallback : public b2QueryCallback {
    QueryCallback(ReportFixtureCB reportFixture, ReportParticleCB reportParticle, ShouldQueryParticleSystemCB shouldQueryParticleSystem)
        : m_reportFixture(*reportFixture),
          m_reportParticle(*reportParticle),
          m_shouldQueryParticleSystem(*shouldQueryParticleSystem) {}

    ~QueryCallback() {}

    bool ReportFixture(b2Fixture* fixture) override {
        return m_reportFixture(fixture);
    }

    bool ReportParticle(const b2ParticleSystem* particleSystem, int32 index) override {
        return m_reportParticle(particleSystem, index);
    }

    bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override {
        return m_shouldQueryParticleSystem(particleSystem);
    }

    ReportFixtureCB m_reportFixture;
    ReportParticleCB m_reportParticle;
    ShouldQueryParticleSystemCB m_shouldQueryParticleSystem;
};

EXPORT QueryCallback* QueryCallback_new(ReportFixtureCB reportFixture, ReportParticleCB reportParticle, ShouldQueryParticleSystemCB shouldQueryParticleSystem);
EXPORT QueryCallback QueryCallback_create(ReportFixtureCB reportFixture, ReportParticleCB reportParticle, ShouldQueryParticleSystemCB shouldQueryParticleSystem);

typedef float (*ReportFixtureRayCastCB)(b2Fixture*, b2Vec2, b2Vec2, float);
typedef float32 (*ReportParticleRayCastCB)(const b2ParticleSystem*, int32, const b2Vec2*, const b2Vec2*, float32);
typedef bool (*ShouldQueryParticleSystemRayCastCB)(const b2ParticleSystem*);

struct RayCastCallback : public b2RayCastCallback {
    RayCastCallback(ReportFixtureRayCastCB reportFixture, ReportParticleRayCastCB reportParticle, ShouldQueryParticleSystemRayCastCB shouldQueryParticleSystem)
        : m_reportFixture(*reportFixture),
          m_reportParticle(*reportParticle),
          m_shouldQueryParticleSystem(*shouldQueryParticleSystem) {}

    ~RayCastCallback() {}

    float ReportFixture(b2Fixture* fixture, const b2Vec2& point, const b2Vec2& normal, float fraction) override {
        return m_reportFixture(fixture, point, normal, fraction);
    }

    float32 ReportParticle(const b2ParticleSystem* particleSystem, int32 index, const b2Vec2& point, const b2Vec2& normal, float32 fraction) override {
        return m_reportParticle(particleSystem, index, &point, &normal, fraction);
    }

    bool ShouldQueryParticleSystem(const b2ParticleSystem* particleSystem) override {
        return m_shouldQueryParticleSystem(particleSystem);
    }

    ReportFixtureRayCastCB m_reportFixture;
    ReportParticleRayCastCB m_reportParticle;
    ShouldQueryParticleSystemRayCastCB m_shouldQueryParticleSystem;
};

EXPORT RayCastCallback* RayCastCallback_new(ReportFixtureRayCastCB reportFixture, ReportParticleRayCastCB reportParticle, ShouldQueryParticleSystemRayCastCB shouldQueryParticleSystem);
EXPORT RayCastCallback RayCastCallback_create(ReportFixtureRayCastCB reportFixture, ReportParticleRayCastCB reportParticle, ShouldQueryParticleSystemRayCastCB shouldQueryParticleSystem);
