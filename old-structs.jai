b2_blockSizeCount :: 14;
b2_maxManifoldPoints :: 2;
b2_lengthUnitsPerMeter :: 1.0;
b2_maxPolygonVertices :: 8;

b2AABB :: struct {
    lowerBound: Vector2;
    upperBound: Vector2;
}

b2BlockAllocator :: struct {
	m_chunks: *b2Chunk;
    m_chunkCount: s32;
    m_chunkSpace: s32;

	m_freeLists: *b2Block = [b2_blockSizeCount] b2Block;
}

b2BodyDef :: struct {
    type: b2BodyType = b2_staticBody;
    position: Vector2;
    angle: float;
    linearVelocity: Vector2;
    angularVelocity: float;
    linearDamping: float;
    angularDamping: float;
    allowSleep: bool = true;
    awake: bool = true;
    fixedRotation: bool;
    bullet: bool;
    enabled: bool = true;
    userData: b2BodyUserData;
    gravityScale: float = 1.0;
}

b2BodyUserData :: struct {
	pointer: u64;
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

b2ChainShape :: struct {
    using shape: b2Shape;
    
	m_vertices: *Vector2;
	m_count: s32;
	m_prevVertex: Vector2;
    m_nextVertex: Vector2;
}

b2CircleShape :: struct {
    using shape: b2Shape;
    m_p: Vector2;
}

b2ClipVertex :: struct {
	v: Vector2;
	id: b2ContactID;
}

b2Color :: struct {
	r: float;
    g: float;
    b: float;
    a: float;
}

b2Contact :: struct {
	m_flags: uint32;

	m_prev: *b2Contact;
	m_next: *b2Contact;

	m_nodeA: b2ContactEdge;
	m_nodeB: b2ContactEdge;

	m_fixtureA: *b2Fixture;
	m_fixtureB: *b2Fixture;

	m_indexA: int32;
	m_indexB: int32;

	m_manifold: b2Manifold;

	m_toiCount: int32;
	m_toi: float;

	m_friction: float;
	m_restitution: float;
	m_restitutionThreshold: float;

	m_tangentSpeed: float;
}

b2ContactFilter :: struct {

}

b2ContactListener :: struct {

}

b2ContactManager :: struct {
	m_broadPhase: b2BroadPhase;
	m_contactList: *b2Contact;
	m_contactCount: int32;
	m_contactFilter: *b2ContactFilter;
	m_contactListener: *b2ContactListener;
	m_allocator: *b2BlockAllocator;
}

b2DestructionListener :: struct {

}

b2DistanceInput :: struct {
	proxyA: b2DistanceProxy;
	proxyB: b2DistanceProxy;
	transformA: b2Transform;
	transformB: b2Transform;
	useRadii: bool;
}

b2DistanceJoint :: struct {
    using joint: b2Joint;
    
	m_stiffness: float;
	m_damping: float;
	m_bias: float;
	m_length: float;
	m_minLength: float;
	m_maxLength: float;

	m_localAnchorA: b2Vec2;
	m_localAnchorB: b2Vec2;
	m_gamma: float;
	m_impulse: float;
	m_lowerImpulse: float;
	m_upperImpulse: float;

	m_indexA: int32;
	m_indexB: int32;
	m_u: b2Vec2;
	m_rA: b2Vec2;
	m_rB: b2Vec2;
	m_localCenterA: b2Vec2;
	m_localCenterB: b2Vec2;
	m_currentLength: float;
	m_invMassA: float;
	m_invMassB: float;
	m_invIA: float;
	m_invIB: float;
	m_softMass: float;
	m_mass: float;
}

b2DistanceJointDef :: struct {
    using joint: b2Joint;
    
	localAnchorA: b2Vec2;

	localAnchorB: b2Vec2;

	length: float;

	minLength: float;

	maxLength: float;

	stiffness: float;

	damping: float;
}

b2DistanceOutput :: struct {
	pointA: b2Vec2;
	pointB: b2Vec2;
	distance: float;
	iterations: int32;
}

b2DistanceProxy :: struct {
	m_buffer: [2] b2Vec2;
	m_vertices: *b2Vec2;
	m_count: int32;
	m_radius: float;
}

b2Draw :: struct {
	m_drawFlags: b2DrawFlag;
}

b2DrawFlag :: enum {
    e_shapeBit :: 0x0001;
    e_jointBit :: 0x0002;
    e_aabbBit :: 0x0004;
    e_pairBit :: 0x0008;
    e_centerOfMassBit :: 0x0010;
}

b2DynamicTree :: struct {
	m_root: int32;

	m_nodes: *b2TreeNode;
	m_nodeCount: int32;
	m_nodeCapacity: int32;

	m_freeList: int32;

	m_insertionCount: int32;
}

b2EdgeShape :: struct {
    using shape: b2Shape;
    
	m_vertex1: Vector2;
    m_vertex2: Vector2;

	m_vertex0: Vector2;
    m_vertex3: Vector2;

	m_oneSided: bool;
}

b2Filter :: struct {
	categoryBits: u16 = 0x0001;
	maskBits: u16 = 0xFFFF;
	groupIndex: i16;
}

b2FixtureDef :: struct {
	shape: *b2Shape;

	userData: b2FixtureUserData;

	friction: float = 0.2;

	restitution: float;

	restitutionThreshold: float = 1.0 * b2_lengthUnitsPerMeter;

	density: float;

	isSensor: bool;

	filter: b2Filter;
}

b2FixtureProxy :: struct {
	aabb: b2AABB;
	fixture: *b2Fixture;
	childIndex: int32;
	proxyId: int32;
}

b2FixtureUserData :: struct {
    pointer: u64;
}

b2FrictionJoint :: struct {
    using joint: b2Joint;

	m_localAnchorA: b2Vec2;
	m_localAnchorB: b2Vec2;

		m_linearImpulse: b2Vec2;
	m_angularImpulse: float;
	m_maxForce: float;
	m_maxTorque: float;

		m_indexA: int32;
	m_indexB: int32;
	m_rA: b2Vec2;
	m_rB: b2Vec2;
	m_localCenterA: b2Vec2;
	m_localCenterB: b2Vec2;
	m_invMassA: float;
	m_invMassB: float;
	m_invIA: float;
	m_invIB: float;
	m_linearMass: b2Mat22;
	m_angularMass: float;
}

b2FrictionJointDef :: struct {
    using def: b2JointDef;

	localAnchorA: b2Vec2;

	localAnchorB: b2Vec2;

	maxForce: float;

	maxTorque: float;
}

b2GearJoint :: struct {
    using joint: b2Joint;

	m_joint1: *b2Joint;
	m_joint2: *b2Joint;

	m_typeA: b2JointType;
	m_typeB: b2JointType;

			m_bodyC: b2Body*;
	m_bodyD: b2Body*;

		m_localAnchorA: b2Vec2;
	m_localAnchorB: b2Vec2;
	m_localAnchorC: b2Vec2;
	m_localAnchorD: b2Vec2;

	m_localAxisC: b2Vec2;
	m_localAxisD: b2Vec2;

	m_referenceAngleA: float;
	m_referenceAngleB: float;

	m_constant: float;
	m_ratio: float;

	m_impulse: float;

		m_indexA, m_indexB, m_indexC, m_indexD: int32;
	m_lcA, m_lcB, m_lcC, m_lcD: b2Vec2;
	m_mA, m_mB, m_mC, m_mD: float;
	m_iA, m_iB, m_iC, m_iD: float;
	m_JvAC, m_JvBD: b2Vec2;
	m_JwA, m_JwB, m_JwC, m_JwD: float;
	m_mass: float;
}

b2GearJointDef :: struct {
    using def: b2JointDef;
    
	/	joint1: *b2Joint;

	/	joint2: *b2Joint;

	/	/	ratio: float;
}

b2Jacobian :: struct { 
	linear: b2Vec2;
	angularA: float;
	angularB: float;
}

b2Joint :: struct {
	m_type: b2JointType;
	m_prev: *b2Joint;
	m_next: *b2Joint;
	m_edgeA: b2JointEdge;
	m_edgeB: b2JointEdge;
	m_bodyA: *b2Body;
	m_bodyB: *b2Body;

	m_index: int32;

	m_islandFlag: bool;
	m_collideConnected: bool;

	m_userData: b2JointUserData;
}

b2JointDef :: struct {
	/	b2JointType type;

	/	b2JointUserData userData;

	/	b2Body* bodyA;

	/	b2Body* bodyB;

	/	bool collideConnected;
}

b2JointType :: enum {
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
	e_motorJoin;
}

b2JointUserData :: struct {
    pointer: u64;
}

b2Manifold :: struct {
	points: [b2_maxManifoldPoints] b2ManifoldPoint;
	localNormal: b2Vec2;
	localPoint: b2Vec2;
	type: b2ManifoldType
	pointCount: int32;
}

b2ManifoldType :: enum {
    e_circles;
    e_faceA;
    e_faceB;
}

b2ManifoldPoint :: struct {
	localPoint: b2Vec2;
	normalImpulse: float;
	tangentImpulse: float;
	id: b2ContactID;
}

b2MassData :: struct {
	mass: float;

	center: b2Vec2;

	I: float;
}

b2Mat22 :: struct {
	ex, ey: Vector2;
}

b2Mat33 :: struct {
	ex, ey, ez: Vector3;
}

b2MotorJoint :: struct {
    using joint: b2Joint;
    
	m_linearOffset: b2Vec2;
	m_angularOffset: float;
	m_linearImpulse: b2Vec2;
	m_angularImpulse: float;
	m_maxForce: float;
	m_maxTorque: float;
	m_correctionFactor: float;

	m_indexA: int32;
	m_indexB: int32;
	m_rA: b2Vec2;
	m_rB: b2Vec2;
	m_localCenterA: b2Vec2;
	m_localCenterB: b2Vec2;
	m_linearError: b2Vec2;
	m_angularError: float;
	m_invMassA: float;
	m_invMassB: float;
	m_invIA: float;
	m_invIB: float;
	m_linearMass: b2Mat22;
	m_angularMass: float;
}

b2MotorJointDef :: struct {
    using def: b2JointDef;

	linearOffset: b2Vec2;
	angularOffset: float;
	maxForce: float;
	maxTorque: float;
	correctionFactor: float;
}

b2MouseJoint :: struct {
    using joint: b2Joint;

	m_localAnchorB: b2Vec2;
	m_targetA: b2Vec2;
	m_stiffness: float;
	m_damping: float;
	m_beta: float;

		m_impulse: b2Vec2;
	m_maxForce: float;
	m_gamma: float;

		m_indexA: int32;
	m_indexB: int32;
	m_rB: b2Vec2;
	m_localCenterB: b2Vec2;
	m_invMassB: float;
	m_invIB: float;
	m_mass: b2Mat22;
	m_C: b2Vec2;
}

b2MouseJointDef :: struct {
    using def: b2JointDef;
    
	target: b2Vec2;
	maxForce: float;
	stiffness: float;
	damping: float;
}

b2PolygonShape :: struct {
    using shape: b2Shape;
    
	m_centroid: b2Vec2;
	m_vertices: [b2_maxPolygonVertices] b2Vec2;
	m_normals: [b2_maxPolygonVertices] b2Vec2;
	m_count: int32;
}

b2Position :: struct {
	c: b2Vec2;
	a: float;
}

b2PrismaticJoint :: struct {
    using joint: b2Joint;

	m_localAnchorA: b2Vec2;
	m_localAnchorB: b2Vec2;
	m_localXAxisA: b2Vec2;
	m_localYAxisA: b2Vec2;
	m_referenceAngle: float;
	m_impulse: b2Vec2;
	m_motorImpulse: float;
	m_lowerImpulse: float;
	m_upperImpulse: float;
	m_lowerTranslation: float;
	m_upperTranslation: float;
	m_maxMotorForce: float;
	m_motorSpeed: float;
	m_enableLimit: bool;
	m_enableMotor: bool;

		m_indexA: int32;
	m_indexB: int32;
	m_localCenterA: b2Vec2;
	m_localCenterB: b2Vec2;
	m_invMassA: float;
	m_invMassB: float;
	m_invIA: float;
	m_invIB: float;
	m_axis, m_perp: b2Vec2;
	m_s1, m_s2: float;
	m_a1, m_a2: float;
	m_K: b2Mat22;
	m_translation: float;
	m_axialMass: float;
}

b2PrismaticJointDef :: struct {
    using def: b2JointDef;

	localAnchorA: b2Vec2;
	localAnchorB: b2Vec2;
	localAxisA: b2Vec2;
	referenceAngle: float;
	enableLimit: bool;
	lowerTranslation: float;
	upperTranslation: float;
	enableMotor: bool;
	maxMotorForce: float;
	motorSpeed: float;
}

b2PulleyJoint :: struct {
    using joint: b2Joint;

	m_groundAnchorA: b2Vec2;
	m_groundAnchorB: b2Vec2;
	m_lengthA: float;
	m_lengthB: float;

		m_localAnchorA: b2Vec2;
	m_localAnchorB: b2Vec2;
	m_constant: float;
	m_ratio: float;
	m_impulse: float;

		m_indexA: int32;
	m_indexB: int32;
	m_uA: b2Vec2;
	m_uB: b2Vec2;
	m_rA: b2Vec2;
	m_rB: b2Vec2;
	m_localCenterA: b2Vec2;
	m_localCenterB: b2Vec2;
	m_invMassA: float;
	m_invMassB: float;
	m_invIA: float;
	m_invIB: float;
	m_mass: float;
}

b2PulleyJointDef :: struct {
    using def: b2JointDef;

	groundAnchorA: b2Vec2;
	groundAnchorB: b2Vec2;
	localAnchorA: b2Vec2;
	localAnchorB: b2Vec2;
	lengthA: float;
	lengthB: float;
	ratio: float;
}

b2QueryCallback :: struct {

}

b2Shape :: struct {
    m_type: b2ShapeType;
    m_radius: float;
}

b2ShapeType :: enum {
    e_circle :: 0;
    e_edge;
    e_polygon;
    e_chain;
    e_typeCount;
}

b2Body :: struct {
	m_type: b2BodyType;

	m_flags: u16;

	m_islandIndex: s32;

	m_xf: b2Transform;			m_sweep: b2Sweep;		
	m_linearVelocity: Vector2;
	m_angularVelocity: float;

	m_force: b2Vec2;
	m_torque: float;

	m_world: *b2World;
	m_prev: *b2Body;
	m_next: *b2Body;

	m_fixtureList: *b2Fixture;
	m_fixtureCount: s32;

	m_jointList: *b2JointEdge;
	m_contactList: *b2ContactEdge;

	m_mass: float;
    m_invMass: float;

	m_I: float;
    m_invI: float;

	m_linearDamping: float;
	m_angularDamping: float;
	m_gravityScale: float;

	m_sleepTime: float;

	m_userData: b2BodyUserData;
}

b2BodyType :: enum {
	b2_staticBody :: 0;
	b2_kinematicBody;
	b2_dynamicBody;
}

b2World :: struct {
	m_blockAllocator: b2BlockAllocator;
	m_stackAllocator: b2StackAllocator;

	m_contactManager: b2ContactManager;

	m_bodyList: *b2Body;
	m_jointList: *b2Joint;

	m_bodyCount: int32;
	m_jointCount: int32;

	m_gravity: b2Vec2;
	m_allowSleep: bool;

	m_destructionListener: *b2DestructionListener;
	m_debugDraw: *b2Draw;

			m_inv_dt0: float;

	m_newContacts: bool;
	m_locked: bool;
	m_clearForces: bool;

		m_warmStarting: bool;
	m_continuousPhysics: bool;
	m_subStepping: bool;

	m_stepComplete: bool;

	m_profile: b2Profile;
}

b2Profile :: struct {
	step: float;
	collide: float;
	solve: float;
	solveInit: float;
	solveVelocity: float;
	solvePosition: float;
	broadphase: float;
	solveTOI: float;
}

b2Transform :: struct {
	p: Vector2;
	q: b2Rot;
}

b2WorldManifold :: struct {
	normal: Vector2;
	points: [b2_maxManifoldPoints] Vector2;
	separations: [b2_maxManifoldPoints] float;
}
