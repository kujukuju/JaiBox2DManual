b2_maxManifoldPoints :: 2;
b2_maxPolygonVertices :: 8;
b2_blockSizes :: 14;
b2_stackSize :: 100 * 1024;
b2_maxStackEntries :: 32;

// TODO try to use jai mats I think
b2Mat22 :: struct {
    ex: Vector2;
    ey: Vector2;
}

b2Mat33 :: struct {
    ex: Vector3;
    ey: Vector3;
    ez: Vector3;
}

b2WorldManifold :: struct {
    normal: Vector2;
    points: [b2_maxManifoldPoints] Vector2;
    separations: [b2_maxManifoldPoints] float32;
}

b2ManifoldPoint :: struct {
    localPoint: Vector2;
    normalImpulse: float32;
    tangentImpulse: float32;
    id: b2ContactID;
}

b2ManifoldType :: enum u32 {
    e_circles;
    e_faceA;
    e_faceB;
}

b2Manifold :: struct {
    points: [b2_maxManifoldPoints] b2ManifoldPoint;
    localNormal: Vector2;
    localPoint: Vector2;
    type: b2ManifoldType;
    pointCount: s32;
}

b2ContactFeature :: struct {
    indexA: u8;
    indexB: u8;
    typeA: u8;
    typeB: u8;
}

b2Rot :: struct {
    s: float32;
    c: float32;
}

b2ContactID :: union {
    cf: b2ContactFeature;
    key: u32;
}

b2Transform :: struct {
	p: Vector2;
	q: b2Rot;
}

b2PointState :: enum u32 {
    b2_nullState;
    b2_addState;
    b2_persistState;
    b2_removeState;
}

b2ShapeType :: enum u32 {
    e_circle :: 0;
    e_edge :: 1;
    e_polygon :: 2;
    e_chain :: 3;
    e_typeCount :: 4;
}

b2Shape :: struct {
    vtable: *void;
    m_type: b2ShapeType;
    m_radius: float32;
}

b2Sweep :: struct {
    localCenter: Vector2;
    c0: Vector2;
    c: Vector2;
    a0: float32;
    a: float32;
    alpha0: float32;
}

b2ContactImpulse :: struct {
    normalImpulses: [b2_maxManifoldPoints] float32;
    tangentImpulses: [b2_maxManifoldPoints] float32;
    count: s32;
}

b2Contact :: struct {
    vtable: u64;
    m_flags: u32;
    m_prev: *b2Contact;
    m_next: *b2Contact;
    m_nodeA: b2ContactEdge;
    m_nodeB: b2ContactEdge;
    m_fixtureA: *b2Fixture;
    m_fixtureB: *b2Fixture;
    m_indexA: s32;
    m_indexB: s32;
    m_manifold: b2Manifold;
    m_toiCount: s32;
    m_toi: float32;
    m_friction: float32;
    m_restitution: float32;
    m_tangentSpeed: float32;
}

b2Fixture :: struct {
    m_density: float32;
    m_next: *b2Fixture;
    m_body: *b2Body;
    m_shape: *b2Shape;
    m_friction: float32;
    m_restitution: float32;
    m_proxies: *b2FixtureProxy;
    m_proxyCount: s32;
    m_filter: b2Filter;
    m_isSensor: bool;
    m_userData: *void;
}

b2BodyType :: enum u32 {
	b2_staticBody :: 0;
	b2_kinematicBody;
	b2_dynamicBody;
}

b2Body :: struct {
    m_type: b2BodyType;
    m_flags: u16;
    m_islandIndex: s32;
    m_xf: b2Transform;
    m_sweep: b2Sweep;
    m_linearVelocity: Vector2;
    m_angularVelocity: float32;
    m_force: Vector2;
    m_torque: float32;
    m_world: *b2World;
    m_prev: *b2Body;
    m_next: *b2Body;
    m_fixtureList: *b2Fixture;
    m_fixtureCount: s32;
    m_jointList: *b2JointEdge;
    m_contactList: *b2ContactEdge;
    m_mass: float32;
    m_invMass: float32;
    m_I: float32;
    m_invI: float32;
    m_linearDamping: float32;
    m_angularDamping: float32;
    m_gravityScale: float32;
    m_sleepTime: float32;
    m_userData: *void;
}

b2MassData :: struct {
    mass: float32;
    center: Vector2;
    I: float32;
}

b2JointEdge :: struct {
    other: *b2Body;
    joint: *b2Joint;
    prev: *b2JointEdge;
    next: *b2JointEdge;
}

b2ContactEdge :: struct {
    other: *b2Body;
    contact: *b2Contact;
    prev: *b2ContactEdge;
    next: *b2ContactEdge;
}

b2World :: struct {
    m_blockAllocator: b2BlockAllocator;
    m_stackAllocator: b2StackAllocator;
    m_flags: s32;
    m_contactManager: b2ContactManager;
    m_bodyList: *b2Body;
    m_jointList: *b2Joint;
    m_bodyCount: s32;
    m_jointCount: s32;
    m_gravity: Vector2;
    m_allowSleep: bool;
    m_destructionListener: *b2DestructionListener;
    g_debugDraw: *b2Draw;
    m_inv_dt0: float32;
    m_warmStarting: bool;
    m_continuousPhysics: bool;
    m_subStepping: bool;
    m_stepComplete: bool;
    m_profile: b2Profile;
}

b2FixtureDef :: struct {
    shape: *b2Shape;
    userData: *void;
    friction: float32;
    restitution: float32;
    density: float32;
    isSensor: bool;
    filter: b2Filter;
}

b2FixtureProxy :: struct {
    aabb: b2AABB;
    fixture: *b2Fixture;
    childIndex: s32;
    proxyId: s32;
}

b2Block :: struct {
    next: *b2Block;
}

b2Chunk :: struct {
    blockSize: s32;
    blocks: *b2Block;
}

b2BlockAllocator :: struct {
    m_chunks: *b2Chunk;
    m_chunkCount: s32;
    m_chunkSpace: s32;
    m_freeLists: [b2_blockSizes] *b2Block;
}

b2StackAllocator :: struct {
    m_data: [b2_stackSize] u8;
    m_index: s32;
    m_allocation: s32;
    m_maxAllocation: s32;
    m_entries: [b2_maxStackEntries] b2StackEntry;
    m_entryCount: s32;
}

b2StackEntry :: struct {
    data: *u8;
    size: s32;
    usedMalloc: bool;
}

b2Filter :: struct {
    categoryBits: u16;
    maskBits: u16;
    groupIndex: s16;
}

b2RayCastOutput :: struct {
	normal: Vector2;
	fraction: float32;
}

b2RayCastInput :: struct {
    p1: Vector2;
    p2: Vector2;
    maxFraction: float32;
}

b2AABB :: struct {
    lowerBound: Vector2;
    upperBound: Vector2;
}

b2DestructionListener :: struct {
    vtable: *void;
}

b2ContactFilter :: struct {
    vtable: *void;
}

b2ContactListener :: struct {
    vtable: *void;
}

b2Draw :: struct {
    vtable: *void;
    m_drawFlags: u32;
}

b2BodyDef :: struct {
    type: b2BodyType;
    position: Vector2;
    angle: float32;
    linearVelocity: Vector2;
    angularVelocity: float32;
    linearDamping: float32;
    angularDamping: float32;
    allowSleep: bool;
    awake: bool;
    fixedRotation: bool;
    bullet: bool;
    active: bool;
    userData: *void;
    gravityScale: float32;
}

b2JointType :: enum u32 {
    e_unknownJoint;
	e_revoluteJoint;
	e_prismaticJoint;
	e_distanceJoint;
	e_pulleyJoint;
	e_mouseJoint;
	e_gearJoint;
	e_wheelJoint;
    e_weldJoint;
	e_frictionJoint;
	e_ropeJoint;
	e_motorJoint;
}

b2Joint :: struct {
    vtable: *void;
    m_type: b2JointType;
    m_prev: *b2Joint;
    m_next: *b2Joint;
    m_edgeA: b2JointEdge;
    m_edgeB: b2JointEdge;
    m_bodyA: *b2Body;
    m_bodyB: *b2Body;
    m_index: s32;
    m_islandFlag: bool;
    m_collideConnected: bool;
    m_userData: *void;
}

b2JointDef :: struct {
    type: b2JointType;
    userData: *void;
    bodyA: *b2Body;
    bodyB: *b2Body;
    collideConnected: bool;
}

b2QueryCallback :: struct {
    vtable: *void;
}

b2RayCastCallback :: struct {
    vtable: *void;
}

b2DynamicTree :: struct {
    m_root: s32;
    m_nodes: *b2TreeNode;
    m_nodeCount: s32;
    m_nodeCapacity: s32;
    m_freeList: s32;
    m_path: u32;
    m_insertionCount: s32;
}

b2TreeNode :: struct {
    aabb: b2AABB;
    userData: *void;
    union {
        parent: s32;
        next: s32;
    }
    child1: s32;
    child2: s32;
    height: s32;
}

b2Pair :: struct {
    proxyIdA: s32;
    proxyIdB: s32;
}

b2BroadPhase :: struct {
    m_tree: b2DynamicTree;
    m_proxyCount: s32;
    m_moveBuffer: *s32;
    m_moveCapacity: s32;
    m_moveCount: s32;
    m_pairBuffer: *b2Pair;
    m_pairCapacity: s32;
    m_pairCount: s32;
    m_queryProxyId: s32;
}

b2ContactManager :: struct {
    m_broadPhase: b2BroadPhase;
    m_contactList: *b2Contact;
    m_contactCount: s32;
    m_contactFilter: *b2ContactFilter;
    m_contactListener: *b2ContactListener;
    m_allocator: *b2BlockAllocator;
}

b2Profile :: struct {
    step: float32;
    collide: float32;
    solve: float32;
    solveInit: float32;
    solveVelocity: float32;
    solvePosition: float32;
    broadphase: float32;
    solveTOI: float32;
}

b2ChainShape :: struct {
    using shape: b2Shape;

    m_vertices: *Vector2;
    m_count: s32;
    m_prevVertex: Vector2;
    m_nextVertex: Vector2;
    m_hasPrevVertex: bool;
    m_hasNextVertex: bool;
}

b2EdgeShape :: struct {
    using shape: b2Shape;

    m_vertex1: Vector2;
    m_vertex2: Vector2;
    m_vertex0: Vector2;
    m_vertex3: Vector2;
    m_hasVertex0: bool;
    m_hasVertex3: bool;
}

b2CircleShape :: struct {
    using shape: b2Shape;

    m_p: Vector2;
}

b2PolygonShape :: struct {
    using shape: b2Shape;

    m_centroid: Vector2;
    m_vertices: [b2_maxPolygonVertices] Vector2;
    m_normals: [b2_maxPolygonVertices] Vector2;
    m_count: s32;
}

b2DistanceJointDef :: struct {
    using joint_def: b2JointDef;

    localAnchorA: Vector2;
    localAnchorB: Vector2;
    length: float32;
    frequencyHz: float32;
    dampingRatio: float32;
}

b2DistanceJoint :: struct {
    using joint: b2Joint;

    m_frequencyHz: float32;
    m_dampingRatio: float32;
    m_bias: float32;
    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_gamma: float32;
    m_impulse: float32;
    m_length: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_u: Vector2;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_mass: float32;
}

b2FrictionJointDef :: struct {
    using joint_def: b2JointDef;

    localAnchorA: Vector2;
    localAnchorB: Vector2;
    maxForce: float32;
    maxTorque: float32;
}

b2FrictionJoint :: struct {
    using joint: b2Joint;

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_linearImpulse: Vector2;
    m_angularImpulse: float32;
    m_maxForce: float32;
    m_maxTorque: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_linearMass: b2Mat22;
    m_angularMass: float32;
}

b2GearJoint :: struct {
    using joint: b2Joint;

    m_joint1: *b2Joint;
    m_joint2: *b2Joint;
    m_typeA: b2JointType;
    m_typeB: b2JointType;
    m_bodyC: *b2Body;
    m_bodyD: *b2Body;
    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_localAnchorC: Vector2;
    m_localAnchorD: Vector2;
    m_localAxisC: Vector2;
    m_localAxisD: Vector2;
    m_referenceAngleA: float32;
    m_referenceAngleB: float32;
    m_constant: float32;
    m_ratio: float32;
    m_impulse: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_indexC: s32;
    m_indexD: s32;
    m_lcA: Vector2;
    m_lcB: Vector2;
    m_lcC: Vector2;
    m_lcD: Vector2;
    m_mA: float32;
    m_mB: float32;
    m_mC: float32;
    m_mD: float32;
    m_iA: float32;
    m_iB: float32;
    m_iC: float32;
    m_iD: float32;
    m_JvAC: Vector2;
    m_JvBD: Vector2;
    m_JwA: float32;
    m_JwB: float32;
    m_JwC: float32;
    m_JwD: float32;
    m_mass: float32;
}

b2MotorJointDef :: struct {
    using joint_def: b2JointDef;

    linearOffset: Vector2;
    angularOffset: float32;
    maxForce: float32;
    maxTorque: float32;
    correctionFactor: float32;
}

b2MotorJoint :: struct {
    using joint: b2Joint;

    m_linearOffset: Vector2;
    m_angularOffset: float32;
    m_linearImpulse: Vector2;
    m_angularImpulse: float32;
    m_maxForce: float32;
    m_maxTorque: float32;
    m_correctionFactor: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_linearError: Vector2;
    m_angularError: float32;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_linearMass: b2Mat22;
    m_angularMass: float32;
}

b2MouseJoint :: struct {
    using joint: b2Joint;

    m_localAnchorB: Vector2;
    m_targetA: Vector2;
    m_frequencyHz: float32;
    m_dampingRatio: float32;
    m_beta: float32;
    m_impulse: Vector2;
    m_maxForce: float32;
    m_gamma: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_rB: Vector2;
    m_localCenterB: Vector2;
    m_invMassB: float32;
    m_invIB: float32;
    m_mass: b2Mat22;
    m_C: Vector2;
}

b2PrismaticJointDef :: struct {
    using joint_def: b2JointDef;

    localAnchorA: Vector2;
    localAnchorB: Vector2;
    localAxisA: Vector2;
    referenceAngle: float32;
    enableLimit: bool;
    lowerTranslation: float32;
    upperTranslation: float32;
    enableMotor: bool;
    maxMotorForce: float32;
    motorSpeed: float32;
}

b2PrismaticJoint :: struct {
    using joint: b2Joint;

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_localXAxisA: Vector2;
    m_localYAxisA: Vector2;
    m_referenceAngle: float32;
    m_impulse: Vector3;
    m_motorImpulse: float32;
    m_lowerTranslation: float32;
    m_upperTranslation: float32;
    m_maxMotorForce: float32;
    m_motorSpeed: float32;
    m_enableLimit: bool;
    m_enableMotor: bool;
    m_limitState: b2LimitState;
    m_indexA: s32;
    m_indexB: s32;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_axi: Vector2;
    m_perp: Vector2;
    m_s: float32;
    m_s2: float32;
    m_a: float32;
    m_a2: float32;
    m_K: b2Mat33;
    m_motorMass: float32;
}

b2PulleyJointDef :: struct {
    using joint_def: b2JointDef;

    groundAnchorA: Vector2;
    groundAnchorB: Vector2;
    localAnchorA: Vector2;
    localAnchorB: Vector2;
    lengthA: float32;
    lengthB: float32;
    ratio: float32;
}

b2PulleyJoint :: struct {
    using joint: b2Joint;

    m_groundAnchorA: Vector2;
    m_groundAnchorB: Vector2;
    m_lengthA: float32;
    m_lengthB: float32;
    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_constant: float32;
    m_ratio: float32;
    m_impulse: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_uA: Vector2;
    m_uB: Vector2;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_mass: float32;
}

b2RevoluteJointDef :: struct {
    using joint_def: b2JointDef;

    localAnchorA: Vector2;
    localAnchorB: Vector2;
    referenceAngle: float32;
    enableLimit: bool;
    lowerAngle: float32;
    upperAngle: float32;
    enableMotor: bool;
    motorSpeed: float32;
    maxMotorTorque: float32;
}

b2RevoluteJoint :: struct {
    using joint: b2Joint;

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_impulse: Vector3;
    m_motorImpulse: float32;
    m_enableMotor: bool;
    m_maxMotorTorque: float32;
    m_motorSpeed: float32;
    m_enableLimit: bool;
    m_referenceAngle: float32;
    m_lowerAngle: float32;
    m_upperAngle: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_mass: b2Mat33;
    m_motorMass: float32;
    m_limitState: b2LimitState;
}

b2LimitState :: enum u32 {
	e_inactiveLimit;
	e_atLowerLimit;
	e_atUpperLimit;
	e_equalLimits;
}

b2WeldJointDef :: struct {
    using joint_def: b2JointDef;

    localAnchorA: Vector2;
    localAnchorB: Vector2;
    referenceAngle: float32;
    frequencyHz: float32;
    dampingRatio: float32;
}

b2WeldJoint :: struct {
    using joint: b2Joint;

    m_frequencyHz: float32;
    m_dampingRatio: float32;
    m_bias: float32;
    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_referenceAngle: float32;
    m_gamma: float32;
    m_impulse: Vector3;
    m_indexA: s32;
    m_indexB: s32;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_mass: b2Mat33;
}

b2WheelJointDef :: struct {
    using joint_def: b2JointDef;

    localAnchorA: Vector2;
    localAnchorB: Vector2;
    localAxisA: Vector2;
    enableMotor: bool;
    maxMotorTorque: float32;
    motorSpeed: float32;
    frequencyHz: float32;
    dampingRatio: float32;
}

b2WheelJoint :: struct {
    using joint: b2Joint;

    m_frequencyHz: float32;
    m_dampingRatio: float32;
    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_localXAxisA: Vector2;
    m_localYAxisA: Vector2;
    m_impulse: float32;
    m_motorImpulse: float32;
    m_springImpulse: float32;
    m_maxMotorTorque: float32;
    m_motorSpeed: float32;
    m_enableMotor: bool;
    m_indexA: s32;
    m_indexB: s32;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_ax: Vector2;
    m_ay: Vector2;
    m_sAx: float32;
    m_sBx: float32;
    m_sAy: float32;
    m_sBy: float32;
    m_mass: float32;
    m_motorMass: float32;
    m_springMass: float32;
    m_bias: float32;
    m_gamma: float32;
}

b2ClipVertex :: struct {
    v: Vector2;
    id: b2ContactID;
}

Draw :: struct {
    using draw: b2Draw;
    
    m_drawPolygon: DrawPolygonCB;
    m_drawSolidPolygon: DrawSolidPolygonCB;
    m_drawCircle: DrawCircleCB;
    m_drawSolidCircle: DrawSolidCircleCB;
    m_drawSegment: DrawSegmentCB;
    m_drawTransform: DrawTransformCB;
    m_drawPoint: DrawPointCB;
}

DrawPolygonCB :: #type (vertices: *Vector2, vertexCount: s32, color: b2Color) -> void #c_call;

DrawSolidPolygonCB :: #type (vertices: *Vector2, vertexCount: s32, color: b2Color) -> void #c_call;

DrawCircleCB :: #type (center: Vector2, radius: float32, color: b2Color) -> void #c_call;

DrawSolidCircleCB :: #type (center: Vector2, radius: float32, color: b2Color) -> void #c_call;

DrawSegmentCB :: #type (p1: Vector2, p2: Vector2, color: b2Color) -> void #c_call;

DrawTransformCB :: #type (xf: b2Transform) -> void #c_call;

DrawPointCB :: #type (p: Vector2, size: float, color: b2Color) -> void #c_call;

DestructionListener :: struct {
    using listener: b2DestructionListener;

    m_sayGoodbyeJoint: SayGoodbyeJointCB;
    m_sayGoodbyeFixture: SayGoodbyeFixtureCB;
}

SayGoodbyeJointCB :: #type (joint: *b2Joint) -> void #c_call;

SayGoodbyeFixtureCB :: #type (fixture: *b2Fixture) -> void #c_call;

ContactFilter :: struct {
    using filter: b2ContactFilter;

    m_shouldCollide: ShouldCollideCB;
}

ShouldCollideCB :: #type (fixtureA: *b2Fixture, fixtureB: *b2Fixture) -> bool #c_call;

ContactListener :: struct {
    using listener: b2ContactListener;
    
    m_beginContact: BeginContactCB;
    m_endContact: EndContactCB;
    m_preSolve: PreSolveCB;
    m_postSolve: PostSolveCB;
}

BeginContactCB :: #type (contact: *b2Contact) -> bool #c_call;
EndContactCB :: #type (contact: *b2Contact) -> bool #c_call;
PreSolveCB :: #type (contact: *b2Contact, oldManifold: *b2Manifold) -> bool #c_call;
PostSolveCB :: #type (contact: *b2Contact, impulse: *b2ContactImpulse) -> bool #c_call;

QueryCallback :: struct {
    using callback: b2QueryCallback;
    
    m_reportFixture: ReportFixtureQueryCB;
}

ReportFixtureQueryCB :: #type (fixture: *b2Fixture) -> bool #c_call;

RayCastCallback :: struct {
    using callback: b2RayCastCallback;
    
    m_reportFixture: ReportFixtureRayCastCB;
}

ReportFixtureRayCastCB :: #type (fixture: *b2Fixture, point: Vector2, normal: Vector2, fraction: float) -> float #c_call;

b2GearJointDef :: struct {
    using joint_def: b2JointDef;

    joint1: *b2Joint;
    joint2: *b2Joint;
    ratio: float32;
}

b2MouseJointDef :: struct {
    using joint_def: b2JointDef;

    target: Vector2;
    maxForce: float32;
    stiffness: float32;
    damping: float32;
}

b2RopeTuning :: struct {
    stretchingModel: b2StretchingModel;
    bendingModel: b2BendingModel;
    damping: float32;
    stretchStiffness: float32;
    stretchHertz: float32;
    stretchDamping: float32;
    bendStiffness: float32;
    bendHertz: float32;
    bendDamping: float32;
    isometric: bool;
    fixedEffectiveMass: bool;
    warmStart: bool;
}

b2RopeDef :: struct {
    position: Vector2;
    vertices: *Vector2;
    count: s32;
    masses: *float32;
    gravity: Vector2;
    tuning: b2RopeTuning;
}

b2Rope :: struct {
    m_position: Vector2;
    m_count: s32;
    m_stretchCount: s32;
    m_bendCount: s32;
    m_stretchConstraints: *b2RopeStretch;
    m_bendConstraints: *b2RopeBend;
    m_bindPositions: *Vector2;
    m_ps: *Vector2;
    m_p0s: *Vector2;
    m_vs: *Vector2;
    m_invMasses: *float;
    m_gravity: Vector2;
    m_tuning: b2RopeTuning;
}

b2StretchingModel :: enum u32 {
    b2_pbdStretchingModel;
    b2_xpbdStretchingModel;
}

b2BendingModel :: enum u32 {
    b2_springAngleBendingModel :: 0;
	b2_pbdAngleBendingModel;
	b2_xpbdAngleBendingModel;
	b2_pbdDistanceBendingModel;
	b2_pbdHeightBendingModel;
	b2_pbdTriangleBendingModel;
}

b2RopeStretch :: struct {
	i1: s32;
    i2: s32;
	invMass1: float;
    invMass2: float;
	L: float;
	lambda: float;
	spring: float;
	damper: float;
}

b2RopeBend :: struct {
	i1: s32;
    i2: s32;
    i3: s32;
	invMass1: float;
    invMass2: float;
    invMass3: float;
	invEffectiveMass: float;
	lambda: float;
	L1: float;
    L2: float;
	alpha1: float;
    alpha2: float;
	spring: float;
	damper: float;
}

b2Color :: struct {
    r: float;
    g: float;
    b: float;
    a: float;
}

#scope_file

#import "Math";
