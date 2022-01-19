typedef void (*SayGoodbyeToJointCB)(RustObject, b2Joint*);
typedef void (*SayGoodbyeToFixtureCB)(RustObject, b2Fixture*);

struct DestructionListenerLink: public b2DestructionListener {
    DestructionListenerLink() {}
    ~DestructionListenerLink() {}

    void SayGoodbye(b2Joint* joint) {
        say_goodbye_to_joint(object, joint);
    }
    void SayGoodbye(b2Fixture* fixture) {
        say_goodbye_to_fixture(object, fixture);
    }

    RustObject object;
    SayGoodbyeToJointCB say_goodbye_to_joint;
    SayGoodbyeToFixtureCB say_goodbye_to_fixture;
};

EXPORT DestructionListenerLink* DestructionListenerLink_alloc();
EXPORT void DestructionListenerLink_bind(DestructionListenerLink* self,
                                  RustObject o,
                                  SayGoodbyeToJointCB sgtj,
                                  SayGoodbyeToFixtureCB sgtf);
EXPORT b2DestructionListener* DestructionListenerLink_as_base(DestructionListenerLink* self);
EXPORT void DestructionListenerLink_drop(DestructionListenerLink* self);

typedef bool (*ShouldCollideCB)(RustObject, b2Fixture*, b2Fixture*);

struct ContactFilterLink: public b2ContactFilter {
    ContactFilterLink() {}
    ~ContactFilterLink() {}

    bool ShouldCollide(b2Fixture* fixture_a, b2Fixture* fixture_b) {
        return should_collide(object, fixture_a, fixture_b);
    }

    RustObject object;
    ShouldCollideCB should_collide;
};

EXPORT ContactFilterLink* ContactFilterLink_alloc();
EXPORT void ContactFilterLink_bind(ContactFilterLink* self,
                            RustObject o,
                            ShouldCollideCB sc);
EXPORT b2ContactFilter* ContactFilterLink_as_base(ContactFilterLink* self);
EXPORT void ContactFilterLink_drop(ContactFilterLink* self);

typedef void (*BeginContactCB)(RustObject, b2Contact*);
typedef void (*EndContactCB)(RustObject, b2Contact*);
typedef void (*PreSolveCB)(RustObject, b2Contact*, const b2Manifold*);
typedef void (*PostSolveCB)(RustObject, b2Contact*, const b2ContactImpulse*);

struct ContactListenerLink: public b2ContactListener {
    ContactListenerLink() {}
    ~ContactListenerLink() {}

    void BeginContact(b2Contact* contact) {
        begin_contact(object, contact);
    }
    void EndContact(b2Contact* contact) {
        end_contact(object, contact);
    }
    void PreSolve(b2Contact* contact, const b2Manifold* old_manifold) {
        pre_solve(object, contact, old_manifold);
    }
    void PostSolve(b2Contact* contact, const b2ContactImpulse* impulse) {
        post_solve(object, contact, impulse);
    }

    RustObject object;
    BeginContactCB begin_contact;
    EndContactCB end_contact;
    PreSolveCB pre_solve;
    PostSolveCB post_solve;
};

EXPORT ContactListenerLink* ContactListenerLink_alloc();
EXPORT void ContactListenerLink_bind(ContactListenerLink* self,
                              RustObject o,
                              BeginContactCB bc,
                              EndContactCB ec,
                              PreSolveCB pres,
                              PostSolveCB posts);
EXPORT b2ContactListener* ContactListenerLink_as_base(ContactListenerLink* self);
EXPORT void ContactListenerLink_drop(ContactListenerLink* self);

typedef bool (*QCReportFixtureCB)(RustObject, b2Fixture*);

struct QueryCallbackLink: public b2QueryCallback {
    QueryCallbackLink() {}
    ~QueryCallbackLink() {}

    bool ReportFixture(b2Fixture* fixture) {
        return report_fixture(object, fixture);
    }

    RustObject object;
    QCReportFixtureCB report_fixture;
};

EXPORT QueryCallbackLink* QueryCallbackLink_alloc();
EXPORT void QueryCallbackLink_bind(QueryCallbackLink* self,
                            RustObject object,
                            QCReportFixtureCB rf);
EXPORT b2QueryCallback* QueryCallbackLink_as_base(QueryCallbackLink* self);
EXPORT void QueryCallbackLink_drop(QueryCallbackLink* self);

typedef f32 (*RCCReportFixtureCB)(RustObject, b2Fixture*,
                                  const b2Vec2*, const b2Vec2*, f32);

struct RayCastCallbackLink: public b2RayCastCallback {
    RayCastCallbackLink() {}
    ~RayCastCallbackLink() {}

    f32 ReportFixture(b2Fixture* fixture,
                      const b2Vec2& point,
                      const b2Vec2& normal,
                      f32 fraction) {
        return report_fixture(object, fixture, &point, &normal, fraction);
    }

    RustObject object;
    RCCReportFixtureCB report_fixture;
};

EXPORT RayCastCallbackLink* RayCastCallbackLink_alloc();
EXPORT void RayCastCallbackLink_bind(RayCastCallbackLink* self,
                              RustObject object,
                              RCCReportFixtureCB rf);
EXPORT b2RayCastCallback* RayCastCallbackLink_as_base(RayCastCallbackLink* self);
EXPORT void RayCastCallbackLink_drop(RayCastCallbackLink* self);
