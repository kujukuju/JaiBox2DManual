b2_maxManifoldPoints :: 2;
b2_maxPolygonVertices :: 8;
b2_blockSizeCount :: 14;
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
    m_restitutionThreshold: float32;
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
    m_xf0: b2Transform;
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
    m_contactManager: b2ContactManager;
    m_bodyList: *b2Body;
    m_jointList: *b2Joint;
    m_particleSystemList: *b2ParticleSystem;
    m_bodyCount: s32;
    m_jointCount: s32;
    m_gravity: Vector2;
    m_allowSleep: bool;
    m_destructionListener: *b2DestructionListener;
    m_debugDraw: *b2Draw;
    m_inv_dt0: float32;
    m_newContacts: bool;
    m_locked: bool;
    m_clearForces: bool;
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
    restitutionThreshold: float32;
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
    m_freeLists: [b2_blockSizeCount] *b2Block;
    m_giants: b2TrackedBlockAllocator;
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
    enabled: bool;
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
    moved: bool;
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
    minLength: float32;
    maxLength: float32;
    stiffness: float32;
    damping: float32;
}

b2DistanceJoint :: struct {
    using joint: b2Joint;

    m_stiffness: float32;
    m_damping: float32;
    m_bias: float32;
    m_length: float32;
    m_minLength: float32;
    m_maxLength: float32;
    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_gamma: float32;
    m_impulse: float32;
    m_lowerImpulse: float32;
    m_upperImpulse: float32;
    m_indexA: s32;
    m_indexB: s32;
    m_u: Vector2;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_currentLength: float32;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_softMass: float32;
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
    m_tolerance: float32;
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
    m_stiffness: float32;
    m_damping: float32;
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
    m_lowerImpulse: float32;
    m_upperImpulse: float32;
    m_lowerTranslation: float32;
    m_upperTranslation: float32;
    m_maxMotorForce: float32;
    m_motorSpeed: float32;
    m_enableLimit: bool;
    m_enableMotor: bool;
    
    m_indexA: s32;
    m_indexB: s32;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float32;
    m_invMassB: float32;
    m_invIA: float32;
    m_invIB: float32;
    m_axis: Vector2;
    m_perp: Vector2;
    m_s1: float32;
    m_s2: float32;
    m_a1: float32;
    m_a2: float32;
    m_K: b2Mat22;
    m_translation: float32;
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

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_referenceAngle: float32;
    m_enableLimit: bool;
    m_lowerAngle: float32;
    m_upperAngle: float32;
    m_enableMotor: bool;
    m_motorSpeed: float32;
    m_maxMotorTorque: float32;
}

b2RevoluteJoint :: struct {
    using joint: b2Joint;

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_impulse: Vector2;
    m_motorImpulse: float;
    m_lowerImpulse: float;
    m_upperImpulse: float;
    m_enableMotor: bool;
    m_maxMotorTorque: float;
    m_motorSpeed: float;
    m_enableLimit: bool;
    m_referenceAngle: float;
    m_lowerAngle: float;
    m_upperAngle: float;
    m_indexA: s32;
    m_indexB: s32;
    m_rA: Vector2;
    m_rB: Vector2;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float;
    m_invMassB: float;
    m_invIA: float;
    m_invIB: float;
    m_K: b2Mat22;
    m_angle: float;
    m_axialMass: float;
}

b2LimitState :: enum u32 {
	e_inactiveLimit;
	e_atLowerLimit;
	e_atUpperLimit;
	e_equalLimits;
}

b2WeldJointDef :: struct {
    using joint_def: b2JointDef;

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_referenceAngle: float32;
    m_stiffness: float32;
    m_damping: float32;
}

b2WeldJoint :: struct {
    using joint: b2Joint;

    m_stiffness: float32;
    m_damping: float32;
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

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_localAxisA: Vector2;
    m_enableLimit: bool;
    m_lowerTranslation: float32;
    m_upperTranslation: float32;
    m_enableMotor: bool;
    m_maxMotorTorque: float32;
    m_motorSpeed: float32;
    m_stiffness: float32;
    m_damping: float32;
}

b2WheelJoint :: struct {
    using joint: b2Joint;

    m_localAnchorA: Vector2;
    m_localAnchorB: Vector2;
    m_localXAxisA: Vector2;
    m_localYAxisA: Vector2;
    m_impulse: float;
    m_motorImpulse: float;
    m_springImpulse: float;
    m_lowerImpulse: float;
    m_upperImpulse: float;
    m_translation: float;
    m_lowerTranslation: float;
    m_upperTranslation: float;
    m_maxMotorTorque: float;
    m_motorSpeed: float;
    m_enableLimit: bool;
    m_enableMotor: bool;
    m_stiffness: float;
    m_damping: float;
    m_indexA: s32;
    m_indexB: s32;
    m_localCenterA: Vector2;
    m_localCenterB: Vector2;
    m_invMassA: float;
    m_invMassB: float;
    m_invIA: float;
    m_invIB: float;
    m_ax: Vector2;
    m_ay: Vector2;
    m_sAx: float;
    m_sBx: float;
    m_sAy: float;
    m_sBy: float;
    m_mass: float;
    m_motorMass: float;
    m_axialMass: float;
    m_springMass: float;
    m_bias: float;
    m_gamma: float;
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
    m_drawParticles: DrawParticlesCB;
    m_drawSegment: DrawSegmentCB;
    m_drawTransform: DrawTransformCB;
    m_drawPoint: DrawPointCB;
}

DrawPolygonCB :: #type (vertices: *Vector2, vertexCount: s32, color: b2Color) -> void #c_call;
DrawSolidPolygonCB :: #type (vertices: *Vector2, vertexCount: s32, color: b2Color) -> void #c_call;
DrawCircleCB :: #type (center: Vector2, radius: float32, color: b2Color) -> void #c_call;
DrawSolidCircleCB :: #type (center: Vector2, radius: float32, color: b2Color) -> void #c_call;
DrawParticlesCB :: #type (centers: *Vector2, radius: float32, colors: *b2ParticleColor, count: s32) -> void #c_call;
DrawSegmentCB :: #type (p1: Vector2, p2: Vector2, color: b2Color) -> void #c_call;
DrawTransformCB :: #type (xf: b2Transform) -> void #c_call;
DrawPointCB :: #type (p: Vector2, size: float, color: b2Color) -> void #c_call;

DestructionListener :: struct {
    using listener: b2DestructionListener;

    m_sayGoodbyeJoint: SayGoodbyeJointCB;
    m_sayGoodbyeFixture: SayGoodbyeFixtureCB;
    m_sayGoodbyeParticleGroup: SayGoodbyeParticleGroupCB;
}

SayGoodbyeJointCB :: #type (joint: *b2Joint) -> void #c_call;
SayGoodbyeFixtureCB :: #type (fixture: *b2Fixture) -> void #c_call;
SayGoodbyeParticleGroupCB :: #type (group: *b2ParticleGroup) -> void #c_call;

ContactFilter :: struct {
    using filter: b2ContactFilter;

    m_shouldCollide: ShouldCollideCB;
    m_shouldCollideFixtureParticle: ShouldCollideFixtureParticleCB;
    m_shouldCollideParticleParticle: ShouldCollideParticleParticleCB;
}

ShouldCollideCB :: #type (fixtureA: *b2Fixture, fixtureB: *b2Fixture) -> bool #c_call;
ShouldCollideFixtureParticleCB :: #type (fixture: *b2Fixture, particleSystem: *b2ParticleSystem, particleIndex: s32) -> bool #c_call;
ShouldCollideParticleParticleCB :: #type (particleSystem: *b2ParticleSystem, particleIndexA: s32, particleIndexB: s32) -> bool #c_call;

ContactListener :: struct {
    using listener: b2ContactListener;
    
    m_beginContact: BeginContactCB;
    m_endContact: EndContactCB;
    m_beginContactParticleBody: BeginContactParticleBodyCB;
    m_endContactFixtureParticle: EndContactFixtureParticleCB;
    m_beginContactParticle: BeginContactParticleCB;
    m_endContactParticle: EndContactParticleCB;
    m_preSolve: PreSolveCB;
    m_postSolve: PostSolveCB;
}

BeginContactCB :: #type (contact: *b2Contact) -> void #c_call;
EndContactCB :: #type (contact: *b2Contact) -> void #c_call;
BeginContactParticleBodyCB :: #type (particleSystem: *b2ParticleSystem, particleBodyContact: *b2ParticleBodyContact);
EndContactFixtureParticleCB :: #type (fixture: *b2Fixture, particleSystem: *b2ParticleSystem, index: s32);
BeginContactParticleCB :: #type (particleSystem: *b2ParticleSystem, particleContact: *b2ParticleContact);
EndContactParticleCB :: #type (particleSystem: *b2ParticleSystem, indexA: s32, indexB: s32);
PreSolveCB :: #type (contact: *b2Contact, oldManifold: *b2Manifold) -> void #c_call;
PostSolveCB :: #type (contact: *b2Contact, impulse: *b2ContactImpulse) -> void #c_call;

QueryCallback :: struct {
    using callback: b2QueryCallback;
    
    m_reportFixture: ReportFixtureCB;
    m_reportParticle: ReportParticleCB;
    m_shouldQueryParticleSystem: ShouldQueryParticleSystemCB;
}

ReportFixtureCB :: #type (fixture: *b2Fixture) -> bool #c_call;
ReportParticleCB :: #type (particleSystem: *b2ParticleSystem, index: s32) -> bool #c_call;
ShouldQueryParticleSystemCB :: #type (particleSystem: *b2ParticleSystem) -> bool #c_call;

RayCastCallback :: struct {
    using callback: b2RayCastCallback;
    
    m_reportFixture: ReportFixtureRayCastCB;
    m_reportParticle: ReportParticleRayCastCB;
    m_shouldQueryParticleSystem: ShouldQueryParticleSystemRayCastCB;
}

ReportFixtureRayCastCB :: #type (fixture: *b2Fixture, point: Vector2, normal: Vector2, fraction: float32) -> float32 #c_call;
ReportParticleRayCastCB :: #type (particleSystem: *b2ParticleSystem, index: s32, point: *Vector2, normal: *Vector2, fraction: float32) -> float32 #c_call;
ShouldQueryParticleSystemRayCastCB :: #type (particleSystem: *b2ParticleSystem) -> bool #c_call;

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

b2ParticleSystem :: struct {
    m_paused: bool;
    m_timestamp: s32;
    m_allParticleFlags: s32;
    m_needsUpdateAllParticleFlags: bool;
    m_allGroupFlags: s32;
    m_needsUpdateAllGroupFlags: bool;
    m_hasForce: bool;
    m_iterationIndex: s32;
    m_inverseDensity: float32;
    m_particleDiameter: float32;
    m_inverseDiameter: float32;
    m_squaredDiameter: float32;
    m_count: s32;
    m_internalAllocatedCapacity: s32;
    m_handleAllocator: b2SlabAllocator(b2ParticleHandle);
    m_handleIndexBuffer: UserOverridableBuffer(*b2ParticleHandle);
    m_flagsBuffer: UserOverridableBuffer(u32);
    m_positionBuffer: UserOverridableBuffer(Vector2);
    m_velocityBuffer: UserOverridableBuffer(Vector2);
    m_forceBuffer: *Vector2;
    m_weightBuffer: *float32;
    m_staticPressureBuffer: *float32;
    m_accumulationBuffer: *float32;
    m_accumulation2Buffer: *Vector2;
    m_depthBuffer: *float32;
    m_colorBuffer: UserOverridableBuffer(b2ParticleColor);
    m_groupBuffer: **b2ParticleGroup;
    m_userDataBuffer: UserOverridableBuffer(*void);
    m_stuckThreshold: s32;
    m_lastBodyContactStepBuffer: UserOverridableBuffer(s32);
    m_bodyContactCountBuffer: UserOverridableBuffer(s32);
    m_consecutiveContactStepsBuffer: UserOverridableBuffer(s32);
    m_stuckParticleBuffer: b2GrowableBuffer(s32);
    m_proxyBuffer: b2GrowableBuffer(Proxy);
    m_contactBuffer: b2GrowableBuffer(b2ParticleContact);
    m_bodyContactBuffer: b2GrowableBuffer(b2ParticleBodyContact);
    m_pairBuffer: b2GrowableBuffer(b2ParticlePair);
    m_triadBuffer: b2GrowableBuffer(b2ParticleTriad);
    m_expirationTimeBuffer: UserOverridableBuffer(s32);
    m_indexByExpirationTimeBuffer: UserOverridableBuffer(s32);
    m_timeElapsed: s64;
    m_expirationTimeBufferRequiresSorting: bool;
    m_groupCount: s32;
    m_groupList: *b2ParticleGroup;
    m_def: b2ParticleSystemDef;
    m_world: *b2World;
    m_prev: *b2ParticleSystem;
    m_next: *b2ParticleSystem;
}

b2ParticleSystemDef :: struct {
    strictContactCheck: bool;
    density: float32;
    gravityScale: float32;
    radius: float32;
    maxCount: s32;
    pressureStrength: float32;
    dampingStrength: float32;
    elasticStrength: float32;
    springStrength: float32;
    viscousStrength: float32;
    surfaceTensionPressureStrength: float32;
    surfaceTensionNormalStrength: float32;
    repulsiveStrength: float32;
    powderStrength: float32;
    ejectionStrength: float32;
    staticPressureStrength: float32;
    staticPressureRelaxation: float32;
    staticPressureIterations: s32;
    colorMixingStrength: float32;
    destroyByAge: bool;
    lifetimeGranularity: float32;
}

b2ParticleFlag :: enum u32 {
    b2_waterParticle :: 0;
    b2_zombieParticle :: 1 << 1;
    b2_wallParticle :: 1 << 2;
    b2_springParticle :: 1 << 3;
    b2_elasticParticle :: 1 << 4;
    b2_viscousParticle :: 1 << 5;
    b2_powderParticle :: 1 << 6;
    b2_tensileParticle :: 1 << 7;
    b2_colorMixingParticle :: 1 << 8;
    b2_destructionListenerParticle :: 1 << 9;
    b2_barrierParticle :: 1 << 10;
    b2_staticPressureParticle :: 1 << 11;
    b2_reactiveParticle :: 1 << 12;
    b2_repulsiveParticle :: 1 << 13;
    b2_fixtureContactListenerParticle :: 1 << 14;
    b2_particleContactListenerParticle :: 1 << 15;
    b2_fixtureContactFilterParticle :: 1 << 16;
    b2_particleContactFilterParticle :: 1 << 17;
}

b2ParticleDef :: struct {
    flags: u32;
    position: Vector2;
    velocity: Vector2;
    color: b2ParticleColor;
    lifetime: float32;
    userData: *void;
    group: *b2ParticleGroup;
}

b2ParticleGroupDef :: struct {
    flags: u32;
    groupFlags: u32;
    position: Vector2;
    angle: float32;
    linearVelocity: Vector2;
    angularVelocity: float32;
    color: b2ParticleColor;
    strength: float32;
    shape: *b2Shape;
    shapes: **b2Shape;
    shapeCount: s32;
    stride: float32;
    particleCount: s32;
    positionData: *Vector2;
    lifetime: float32;
    userData: *void;
    group: *b2ParticleGroup;
}

b2ParticleGroup :: struct {
    m_system: *b2ParticleSystem;
    m_firstIndex: s32;
    m_lastIndex: s32;
    m_groupFlags: u32;
    m_strength: float32;
    m_prev: *b2ParticleGroup;
    m_next: *b2ParticleGroup;
    m_timestamp: s32;
    m_mass: float32;
    m_inertia: float32;
    m_center: Vector2;
    m_linearVelocity: Vector2;
    m_angularVelocity: float32;
    m_transform: b2Transform;
    m_userData: *void;
}

b2ParticleContact :: struct {
    indexA: s32;
    indexB: s32;
    weight: float32;
    normal: Vector2;
    flags: u32;
}

b2ParticleHandle :: struct {
    using node: b2TypedIntrusiveListNode;

    m_index: s32;
}

b2ParticleColor :: struct {
    r: u8;
    g: u8;
    b: u8;
    a: u8;
}

b2ParticleBodyContact :: struct {
    index: s32;
    body: *b2Body;
    fixture: *b2Fixture;
    weight: float32;
    normal: Vector2;
    mass: float32;
}

b2ParticlePair :: struct {
    indexA: s32;
    indexB: s32;
    flags: u32;
    strength: float32;
    distance: float32;
}

b2ParticleTriad :: struct {
    indexA: s32;
    indexB: s32;
    indexC: s32;
    flags: u32;
    strength: float32;
    pa: Vector2;
    pb: Vector2;
    pc: Vector2;
    ka: float32;
    kb: float32;
    kc: float32;
    s: float32;
}

b2SlabAllocator :: struct (SlabAllocatorType: Type) {
    m_slabs: b2TrackedBlockAllocator;
    m_itemsPerSlab: u32;
    m_freeList: b2TypedFreeList;
}

UserOverridableBuffer :: struct (UserOverridableBufferType: Type) {
    data: *UserOverridableBufferType;
    userSuppliedCapacity: s32;
}

b2GrowableBuffer :: struct (GrowableBufferType: Type) {
    data: *GrowableBufferType;
    count: s32;
    capacity: s32;
    allocator: *b2BlockAllocator;
}

b2TypedIntrusiveListNode :: struct {
    m_node: b2IntrusiveListNode;
}

b2IntrusiveListNode :: struct {
    m_prev: *b2IntrusiveListNode;
    m_next: *b2IntrusiveListNode;
}

b2TypedFreeList :: struct {
    m_freeList: b2FreeList;
}

b2FreeList :: struct {
    m_allocated: b2IntrusiveListNode;
    m_free: b2IntrusiveListNode;
}

Proxy :: struct {
    index: s32;
    tag: u32;
}

b2TrackedBlockAllocator :: struct {
    m_blocks: b2TypedIntrusiveListNode;
}

b2DrawFlags :: enum u32 {
    e_shapeBit :: 0x0001;
    e_jointBit :: 0x0002;
    e_aabbBit :: 0x0004;
    e_pairBit :: 0x0008;
    e_centerOfMassBit :: 0x0010;
    e_particleBit :: 0x0020;
}

#scope_file

#import "Math";
