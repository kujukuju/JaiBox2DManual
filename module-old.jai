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

b2DrawFlag :: enum u32 {
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

b2ManifoldType :: enum u32 {
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

b2ShapeType :: enum u32 {
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

b2BodyType :: enum u32 {
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

// Body_apply_angular_impulse	0x00000001800020a9	0x000020a9	1 (0x1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_apply_linear_impulse	0x0000000180001532	0x00001532	2 (0x2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_dump	0x0000000180001f5a	0x00001f5a	3 (0x3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_angular_damping	0x0000000180002955	0x00002955	4 (0x4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_contact_list	0x0000000180001825	0x00001825	5 (0x5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_contact_list_const	0x0000000180001b4f	0x00001b4f	6 (0x6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_fixture_list	0x000000018000137a	0x0000137a	7 (0x7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_fixture_list_const	0x0000000180001154	0x00001154	8 (0x8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_gravity_scale	0x0000000180001c53	0x00001c53	9 (0x9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_inertia	0x00000001800014fb	0x000014fb	10 (0xa)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_joint_list	0x0000000180002559	0x00002559	11 (0xb)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_joint_list_const	0x00000001800021cb	0x000021cb	12 (0xc)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_linear_damping	0x0000000180002351	0x00002351	13 (0xd)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_linear_velocity_from_local_point	0x00000001800020e0	0x000020e0	14 (0xe)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_linear_velocity_from_world_point	0x0000000180001c8a	0x00001c8a	15 (0xf)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_local_point	0x0000000180001ae1	0x00001ae1	16 (0x10)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_local_vector	0x000000018000295a	0x0000295a	17 (0x11)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_mass	0x0000000180001172	0x00001172	18 (0x12)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_mass_data	0x000000018000282e	0x0000282e	19 (0x13)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_next	0x0000000180001753	0x00001753	20 (0x14)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_next_const	0x0000000180001d0c	0x00001d0c	21 (0x15)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_type	0x0000000180001bcc	0x00001bcc	22 (0x16)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_user_data	0x00000001800018e8	0x000018e8	23 (0x17)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_world	0x0000000180001cd5	0x00001cd5	24 (0x18)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_world_const	0x0000000180001938	0x00001938	25 (0x19)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_world_point	0x0000000180002720	0x00002720	26 (0x1a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_get_world_vector	0x0000000180002865	0x00002865	27 (0x1b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_is_active	0x0000000180002469	0x00002469	28 (0x1c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_is_awake	0x000000018000237e	0x0000237e	29 (0x1d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_is_bullet	0x0000000180001974	0x00001974	30 (0x1e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_is_fixed_rotation	0x0000000180001cd0	0x00001cd0	31 (0x1f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_is_sleeping_allowed	0x0000000180001131	0x00001131	32 (0x20)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_reset_mass_data	0x0000000180001870	0x00001870	33 (0x21)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_active	0x000000018000277f	0x0000277f	34 (0x22)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_angular_damping	0x0000000180002ab3	0x00002ab3	35 (0x23)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_awake	0x0000000180001352	0x00001352	36 (0x24)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_bullet	0x0000000180001d1b	0x00001d1b	37 (0x25)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_fixed_rotation	0x000000018000185c	0x0000185c	38 (0x26)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_gravity_scale	0x00000001800015f0	0x000015f0	39 (0x27)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_linear_damping	0x0000000180002554	0x00002554	40 (0x28)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_mass_data	0x000000018000184d	0x0000184d	41 (0x29)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_sleeping_allowed	0x000000018000209f	0x0000209f	42 (0x2a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_type	0x0000000180001acd	0x00001acd	43 (0x2b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Body_set_user_data	0x00000001800013e3	0x000013e3	44 (0x2c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_as_shape	0x0000000180002275	0x00002275	45 (0x2d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_clear	0x0000000180001ca3	0x00001ca3	46 (0x2e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_create_chain	0x000000018000119f	0x0000119f	47 (0x2f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_create_loop	0x0000000180001f05	0x00001f05	48 (0x30)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_drop	0x0000000180001f37	0x00001f37	49 (0x31)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_get_child_edge	0x0000000180001799	0x00001799	50 (0x32)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_get_next_vertex	0x0000000180001767	0x00001767	51 (0x33)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_get_prev_vertex	0x0000000180001a7d	0x00001a7d	52 (0x34)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_get_vertex_count	0x0000000180002310	0x00002310	53 (0x35)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_get_vertices_const	0x0000000180001b7c	0x00001b7c	54 (0x36)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_new	0x000000018000145b	0x0000145b	55 (0x37)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_set_next_vertex	0x00000001800022c0	0x000022c0	56 (0x38)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ChainShape_set_prev_vertex	0x00000001800023ba	0x000023ba	57 (0x39)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_as_shape	0x0000000180002306	0x00002306	58 (0x3a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_drop	0x00000001800018d4	0x000018d4	59 (0x3b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_get_pos	0x000000018000268f	0x0000268f	60 (0x3c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_get_support	0x0000000180002202	0x00002202	61 (0x3d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_get_support_vertex	0x00000001800017b7	0x000017b7	62 (0x3e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_get_vertex	0x00000001800017a8	0x000017a8	63 (0x3f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_get_vertex_count	0x000000018000235b	0x0000235b	64 (0x40)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_new	0x00000001800017c1	0x000017c1	65 (0x41)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// CircleShape_set_pos	0x0000000180001618	0x00001618	66 (0x42)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_evaluate_virtual	0x0000000180002842	0x00002842	75 (0x4b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_child_index_a	0x000000018000234c	0x0000234c	76 (0x4c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_child_index_b	0x0000000180001ad2	0x00001ad2	77 (0x4d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_fixture_a	0x0000000180001163	0x00001163	78 (0x4e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_fixture_a_const	0x00000001800028f6	0x000028f6	79 (0x4f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_fixture_b	0x00000001800016fe	0x000016fe	80 (0x50)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_fixture_b_const	0x00000001800015af	0x000015af	81 (0x51)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_friction	0x0000000180002004	0x00002004	82 (0x52)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_manifold	0x000000018000127b	0x0000127b	83 (0x53)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_manifold_const	0x0000000180002793	0x00002793	84 (0x54)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_next	0x0000000180002b12	0x00002b12	85 (0x55)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_next_const	0x0000000180001a5a	0x00001a5a	86 (0x56)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_restitution	0x00000001800029f5	0x000029f5	87 (0x57)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_tangent_speed	0x000000018000160e	0x0000160e	88 (0x58)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_get_world_manifold	0x0000000180001550	0x00001550	89 (0x59)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_is_enabled	0x0000000180001a50	0x00001a50	90 (0x5a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_is_touching	0x0000000180001c67	0x00001c67	91 (0x5b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_reset_friction	0x0000000180002838	0x00002838	92 (0x5c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_reset_restitution	0x00000001800020cc	0x000020cc	93 (0x5d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_set_friction	0x000000018000113b	0x0000113b	94 (0x5e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_set_restitution	0x0000000180001668	0x00001668	95 (0x5f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Contact_set_tangent_speed	0x000000018000123a	0x0000123a	96 (0x60)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactFilterLink_alloc	0x0000000180001fd7	0x00001fd7	67 (0x43)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactFilterLink_as_base	0x00000001800013de	0x000013de	68 (0x44)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactFilterLink_bind	0x0000000180001393	0x00001393	69 (0x45)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactFilterLink_drop	0x0000000180001b5e	0x00001b5e	70 (0x46)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactListenerLink_alloc	0x0000000180001ef6	0x00001ef6	71 (0x47)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactListenerLink_as_base	0x0000000180002a6d	0x00002a6d	72 (0x48)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactListenerLink_bind	0x00000001800025e0	0x000025e0	73 (0x49)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// ContactListenerLink_drop	0x0000000180001d25	0x00001d25	74 (0x4a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DestructionListenerLink_alloc	0x0000000180001e24	0x00001e24	97 (0x61)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DestructionListenerLink_as_base	0x00000001800024ff	0x000024ff	98 (0x62)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DestructionListenerLink_bind	0x00000001800021ad	0x000021ad	99 (0x63)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DestructionListenerLink_drop	0x0000000180001672	0x00001672	100 (0x64)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// distance	0x00000001800027c0	0x000027c0	377 (0x179)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_as_joint	0x000000018000169a	0x0000169a	102 (0x66)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_get_damping_ratio	0x0000000180002112	0x00002112	103 (0x67)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_get_frequency	0x0000000180001285	0x00001285	104 (0x68)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_get_length	0x0000000180001569	0x00001569	105 (0x69)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_get_local_anchor_a	0x0000000180002a54	0x00002a54	106 (0x6a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_get_local_anchor_b	0x0000000180001d7f	0x00001d7f	107 (0x6b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_set_damping_ratio	0x000000018000199c	0x0000199c	108 (0x6c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_set_frequency	0x000000018000165e	0x0000165e	109 (0x6d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJoint_set_length	0x0000000180002135	0x00002135	110 (0x6e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceJointDef_initialize	0x0000000180002905	0x00002905	101 (0x65)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// DistanceProxy_set	0x000000018000144c	0x0000144c	111 (0x6f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_as_shape	0x00000001800026a3	0x000026a3	112 (0x70)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_drop	0x000000018000105f	0x0000105f	113 (0x71)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_get_v0	0x0000000180001faf	0x00001faf	114 (0x72)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_get_v1	0x0000000180002487	0x00002487	115 (0x73)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_get_v2	0x0000000180001488	0x00001488	116 (0x74)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_get_v3	0x000000018000191a	0x0000191a	117 (0x75)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_new	0x000000018000181b	0x0000181b	118 (0x76)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_set	0x0000000180001fb9	0x00001fb9	119 (0x77)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_set_v0	0x00000001800014d8	0x000014d8	120 (0x78)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_set_v1	0x0000000180002991	0x00002991	121 (0x79)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_set_v2	0x0000000180002383	0x00002383	122 (0x7a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// EdgeShape_set_v3	0x000000018000288d	0x0000288d	123 (0x7b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_dump	0x0000000180001064	0x00001064	124 (0x7c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_aabb	0x000000018000293c	0x0000293c	125 (0x7d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_body	0x00000001800028ce	0x000028ce	126 (0x7e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_body_const	0x0000000180001c35	0x00001c35	127 (0x7f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_density	0x0000000180001e2e	0x00001e2e	128 (0x80)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_filter_data	0x0000000180001023	0x00001023	129 (0x81)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_friction	0x000000018000196f	0x0000196f	130 (0x82)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_mass_data	0x0000000180002ae0	0x00002ae0	131 (0x83)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_next	0x00000001800020d1	0x000020d1	132 (0x84)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_next_const	0x00000001800029af	0x000029af	133 (0x85)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_restitution	0x00000001800029e1	0x000029e1	134 (0x86)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_shape	0x00000001800017e4	0x000017e4	135 (0x87)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_shape_const	0x00000001800016cc	0x000016cc	136 (0x88)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_type	0x0000000180001640	0x00001640	137 (0x89)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_get_user_data	0x0000000180001762	0x00001762	138 (0x8a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_is_sensor	0x00000001800019f1	0x000019f1	139 (0x8b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_ray_cast	0x000000018000147e	0x0000147e	140 (0x8c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_refilter	0x0000000180002617	0x00002617	141 (0x8d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_set_density	0x0000000180001f5f	0x00001f5f	142 (0x8e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_set_filter_data	0x0000000180001b3b	0x00001b3b	143 (0x8f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_set_friction	0x00000001800013a7	0x000013a7	144 (0x90)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_set_restitution	0x0000000180001e51	0x00001e51	145 (0x91)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_set_sensor	0x00000001800016d1	0x000016d1	146 (0x92)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_set_user_data	0x000000018000260d	0x0000260d	147 (0x93)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Fixture_test_point	0x0000000180001dc0	0x00001dc0	148 (0x94)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_as_joint	0x00000001800022bb	0x000022bb	150 (0x96)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_get_local_anchor_a	0x00000001800019a6	0x000019a6	151 (0x97)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_get_local_anchor_b	0x0000000180001749	0x00001749	152 (0x98)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_get_max_force	0x0000000180002699	0x00002699	153 (0x99)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_get_max_torque	0x000000018000142e	0x0000142e	154 (0x9a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_set_max_force	0x00000001800021f8	0x000021f8	155 (0x9b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJoint_set_max_torque	0x0000000180002563	0x00002563	156 (0x9c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// FrictionJointDef_initialize	0x0000000180001a5f	0x00001a5f	149 (0x95)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// GearJoint_as_joint	0x0000000180001d39	0x00001d39	157 (0x9d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// GearJoint_get_joint_1	0x00000001800024c3	0x000024c3	158 (0x9e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// GearJoint_get_joint_2	0x00000001800023f1	0x000023f1	159 (0x9f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// GearJoint_get_ratio	0x0000000180001942	0x00001942	160 (0xa0)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// GearJoint_set_ratio	0x0000000180001776	0x00001776	161 (0xa1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// get_point_states	0x0000000180002a86	0x00002a86	378 (0x17a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_distance_joint	0x0000000180001f78	0x00001f78	162 (0xa2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_friction_joint	0x0000000180001ffa	0x00001ffa	163 (0xa3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_gear_joint	0x0000000180002608	0x00002608	164 (0xa4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_motor_joint	0x00000001800019dd	0x000019dd	165 (0xa5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_mouse_joint	0x0000000180001ab4	0x00001ab4	166 (0xa6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_prismatic_joint	0x00000001800016b8	0x000016b8	167 (0xa7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_pulley_joint	0x0000000180001be0	0x00001be0	168 (0xa8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_revolute_joint	0x0000000180001992	0x00001992	169 (0xa9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_rope_joint	0x00000001800028e7	0x000028e7	170 (0xaa)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_weld_joint	0x000000018000240a	0x0000240a	171 (0xab)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_as_wheel_joint	0x000000018000255e	0x0000255e	172 (0xac)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_dump_virtual	0x00000001800016e5	0x000016e5	173 (0xad)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_anchor_a_virtual	0x0000000180001208	0x00001208	174 (0xae)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_anchor_b_virtual	0x0000000180001edd	0x00001edd	175 (0xaf)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_body_a	0x000000018000227f	0x0000227f	176 (0xb0)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_body_b	0x000000018000177b	0x0000177b	177 (0xb1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_collide_connected	0x00000001800029e6	0x000029e6	178 (0xb2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_next	0x0000000180002315	0x00002315	179 (0xb3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_next_const	0x000000018000153c	0x0000153c	180 (0xb4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_reaction_force_virtual	0x00000001800028ab	0x000028ab	181 (0xb5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_reaction_torque_virtual	0x00000001800024a0	0x000024a0	182 (0xb6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_type	0x000000018000193d	0x0000193d	183 (0xb7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_get_user_data	0x00000001800029b9	0x000029b9	184 (0xb8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_is_active	0x0000000180001654	0x00001654	185 (0xb9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_set_user_data	0x0000000180001249	0x00001249	186 (0xba)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Joint_shift_origin_virtual	0x0000000180001b0e	0x00001b0e	187 (0xbb)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_as_joint	0x00000001800021da	0x000021da	189 (0xbd)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_get_angular_offset	0x00000001800024eb	0x000024eb	190 (0xbe)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_get_correction_factor	0x00000001800010fa	0x000010fa	191 (0xbf)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_get_linear_offset	0x0000000180001433	0x00001433	192 (0xc0)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_get_max_force	0x00000001800024e1	0x000024e1	193 (0xc1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_get_max_torque	0x0000000180001a37	0x00001a37	194 (0xc2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_set_angular_offset	0x0000000180002324	0x00002324	195 (0xc3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_set_correction_factor	0x0000000180002022	0x00002022	196 (0xc4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_set_linear_offset	0x0000000180002540	0x00002540	197 (0xc5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_set_max_force	0x00000001800013ac	0x000013ac	198 (0xc6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJoint_set_max_torque	0x0000000180001d98	0x00001d98	199 (0xc7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MotorJointDef_initialize	0x0000000180001604	0x00001604	188 (0xbc)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_as_joint	0x0000000180002036	0x00002036	200 (0xc8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_get_damping_ratio	0x00000001800018cf	0x000018cf	201 (0xc9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_get_frequency	0x000000018000290a	0x0000290a	202 (0xca)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_get_max_force	0x0000000180002482	0x00002482	203 (0xcb)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_get_target	0x00000001800010d2	0x000010d2	204 (0xcc)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_set_damping_ratio	0x0000000180002afe	0x00002afe	205 (0xcd)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_set_frequency	0x000000018000257c	0x0000257c	206 (0xce)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_set_max_force	0x000000018000256d	0x0000256d	207 (0xcf)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// MouseJoint_set_target	0x0000000180001834	0x00001834	208 (0xd0)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_as_shape	0x0000000180001474	0x00001474	209 (0xd1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_drop	0x0000000180002185	0x00002185	210 (0xd2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_get_vertex	0x00000001800020c2	0x000020c2	211 (0xd3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_get_vertex_count	0x0000000180001203	0x00001203	212 (0xd4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_new	0x000000018000295f	0x0000295f	213 (0xd5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_set	0x000000018000110e	0x0000110e	214 (0xd6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_set_as_box	0x00000001800010a0	0x000010a0	215 (0xd7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_set_as_oriented_box	0x0000000180001d3e	0x00001d3e	216 (0xd8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PolygonShape_validate	0x0000000180002144	0x00002144	217 (0xd9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_as_joint	0x0000000180001113	0x00001113	219 (0xdb)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_enable_limit	0x00000001800014e7	0x000014e7	220 (0xdc)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_enable_motor	0x0000000180001fdc	0x00001fdc	221 (0xdd)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_joint_speed	0x000000018000287e	0x0000287e	222 (0xde)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_joint_translation	0x00000001800011a4	0x000011a4	223 (0xdf)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_local_anchor_a	0x000000018000213f	0x0000213f	224 (0xe0)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_local_anchor_b	0x0000000180002437	0x00002437	225 (0xe1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_local_axis_a	0x0000000180001e97	0x00001e97	226 (0xe2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_lower_limit	0x000000018000174e	0x0000174e	227 (0xe3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_max_motor_force	0x000000018000209a	0x0000209a	228 (0xe4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_motor_force	0x000000018000222f	0x0000222f	229 (0xe5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_motor_speed	0x0000000180001dcf	0x00001dcf	230 (0xe6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_reference_angle	0x00000001800020db	0x000020db	231 (0xe7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_get_upper_limit	0x00000001800022fc	0x000022fc	232 (0xe8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_is_limit_enabled	0x00000001800023dd	0x000023dd	233 (0xe9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_is_motor_enabled	0x0000000180001eb5	0x00001eb5	234 (0xea)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_set_limits	0x0000000180002298	0x00002298	235 (0xeb)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_set_max_motor_force	0x0000000180002a4f	0x00002a4f	236 (0xec)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJoint_set_motor_speed	0x0000000180002450	0x00002450	237 (0xed)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PrismaticJointDef_initialize	0x00000001800011c7	0x000011c7	218 (0xda)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_as_joint	0x0000000180002829	0x00002829	239 (0xef)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_current_length_a	0x0000000180002833	0x00002833	240 (0xf0)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_current_length_b	0x0000000180001df7	0x00001df7	241 (0xf1)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_ground_anchor_a	0x0000000180001d93	0x00001d93	242 (0xf2)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_ground_anchor_b	0x00000001800023ec	0x000023ec	243 (0xf3)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_length_a	0x000000018000119a	0x0000119a	244 (0xf4)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_length_b	0x0000000180001fe6	0x00001fe6	245 (0xf5)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJoint_get_ratio	0x0000000180001447	0x00001447	246 (0xf6)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// PulleyJointDef_initialize	0x0000000180001901	0x00001901	238 (0xee)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// QueryCallbackLink_alloc	0x0000000180002289	0x00002289	247 (0xf7)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// QueryCallbackLink_as_base	0x000000018000152d	0x0000152d	248 (0xf8)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// QueryCallbackLink_bind	0x000000018000179e	0x0000179e	249 (0xf9)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// QueryCallbackLink_drop	0x0000000180001375	0x00001375	250 (0xfa)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RayCastCallbackLink_alloc	0x0000000180002923	0x00002923	251 (0xfb)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RayCastCallbackLink_as_base	0x00000001800017fd	0x000017fd	252 (0xfc)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RayCastCallbackLink_bind	0x0000000180002577	0x00002577	253 (0xfd)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RayCastCallbackLink_drop	0x0000000180001f1e	0x00001f1e	254 (0xfe)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_as_joint	0x0000000180001dd9	0x00001dd9	256 (0x100)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_enable_limit	0x0000000180002031	0x00002031	257 (0x101)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_enable_motor	0x00000001800026a8	0x000026a8	258 (0x102)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_joint_angle	0x00000001800012d0	0x000012d0	259 (0x103)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_joint_speed	0x0000000180001c5d	0x00001c5d	260 (0x104)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_local_anchor_a	0x0000000180002b1c	0x00002b1c	261 (0x105)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_local_anchor_b	0x0000000180001dbb	0x00001dbb	262 (0x106)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_lower_limit	0x0000000180001b81	0x00001b81	263 (0x107)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_max_motor_torque	0x0000000180001d20	0x00001d20	264 (0x108)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_motor_speed	0x0000000180001f6e	0x00001f6e	265 (0x109)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_motor_torque	0x000000018000137f	0x0000137f	266 (0x10a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_reference_angle	0x000000018000115e	0x0000115e	267 (0x10b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_get_upper_limit	0x0000000180001e79	0x00001e79	268 (0x10c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_is_limit_enabled	0x00000001800026b2	0x000026b2	269 (0x10d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_is_motor_enabled	0x000000018000157d	0x0000157d	270 (0x10e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_set_limits	0x0000000180002a22	0x00002a22	271 (0x10f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_set_max_motor_torque	0x0000000180001988	0x00001988	272 (0x110)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJoint_set_motor_speed	0x00000001800025a4	0x000025a4	273 (0x111)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RevoluteJointDef_initialize	0x0000000180001b09	0x00001b09	255 (0xff)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RopeJoint_as_joint	0x000000018000261c	0x0000261c	274 (0x112)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RopeJoint_get_limit_state	0x00000001800014f1	0x000014f1	275 (0x113)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RopeJoint_get_local_anchor_a	0x0000000180001a05	0x00001a05	276 (0x114)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RopeJoint_get_local_anchor_b	0x0000000180002667	0x00002667	277 (0x115)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RopeJoint_get_max_length	0x0000000180001217	0x00001217	278 (0x116)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// RopeJoint_set_max_length	0x0000000180001db1	0x00001db1	279 (0x117)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_as_chain_shape	0x00000001800023ab	0x000023ab	280 (0x118)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_as_circle_shape	0x00000001800028b0	0x000028b0	281 (0x119)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_as_edge_shape	0x00000001800019a1	0x000019a1	282 (0x11a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_as_polygon_shape	0x0000000180002b3a	0x00002b3a	283 (0x11b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_compute_aabb_virtual	0x0000000180002257	0x00002257	284 (0x11c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_compute_mass_virtual	0x00000001800029eb	0x000029eb	285 (0x11d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_drop_virtual	0x00000001800013bb	0x000013bb	286 (0x11e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_get_child_count_virtual	0x0000000180001726	0x00001726	287 (0x11f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_get_radius	0x0000000180001276	0x00001276	288 (0x120)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_get_type	0x0000000180001bb8	0x00001bb8	289 (0x121)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_ray_cast_virtual	0x0000000180002595	0x00002595	290 (0x122)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_set_radius	0x000000018000254f	0x0000254f	291 (0x123)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// Shape_test_point_virtual	0x00000001800010a5	0x000010a5	292 (0x124)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// test_overlap	0x000000018000116d	0x0000116d	379 (0x17b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// time_of_impact	0x0000000180002603	0x00002603	380 (0x17c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_as_joint	0x0000000180001bbd	0x00001bbd	294 (0x126)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_get_damping_ratio	0x0000000180001af0	0x00001af0	295 (0x127)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_get_frequency	0x00000001800019e2	0x000019e2	296 (0x128)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_get_local_anchor_a	0x0000000180002b03	0x00002b03	297 (0x129)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_get_local_anchor_b	0x0000000180001cc1	0x00001cc1	298 (0x12a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_get_reference_angle	0x0000000180001d4d	0x00001d4d	299 (0x12b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_set_damping_ratio	0x00000001800027a7	0x000027a7	300 (0x12c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJoint_set_frequency	0x0000000180002973	0x00002973	301 (0x12d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WeldJointDef_initialize	0x000000018000259a	0x0000259a	293 (0x125)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_as_joint	0x00000001800023e7	0x000023e7	303 (0x12f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_enable_motor	0x00000001800025d1	0x000025d1	304 (0x130)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_joint_speed	0x0000000180001005	0x00001005	305 (0x131)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_joint_translation	0x0000000180001a3c	0x00001a3c	306 (0x132)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_local_anchor_a	0x00000001800027d4	0x000027d4	307 (0x133)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_local_anchor_b	0x00000001800016a4	0x000016a4	308 (0x134)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_local_axis_a	0x0000000180002a7c	0x00002a7c	309 (0x135)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_max_motor_torque	0x0000000180001820	0x00001820	310 (0x136)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_motor_speed	0x0000000180002784	0x00002784	311 (0x137)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_motor_torque	0x000000018000198d	0x0000198d	312 (0x138)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_spring_damping_ratio	0x00000001800024cd	0x000024cd	313 (0x139)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_get_spring_frequency	0x00000001800024af	0x000024af	314 (0x13a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_is_motor_enabled	0x000000018000263f	0x0000263f	315 (0x13b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_set_max_motor_torque	0x0000000180001721	0x00001721	316 (0x13c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_set_motor_speed	0x0000000180002419	0x00002419	317 (0x13d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_set_spring_damping_ratio	0x00000001800027cf	0x000027cf	318 (0x13e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJoint_set_spring_frequency	0x00000001800014b5	0x000014b5	319 (0x13f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WheelJointDef_initialize	0x00000001800026d0	0x000026d0	302 (0x12e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_clear_forces	0x0000000180002914	0x00002914	321 (0x141)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_body	0x00000001800022d9	0x000022d9	322 (0x142)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_distance_joint	0x0000000180001979	0x00001979	323 (0x143)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_friction_joint	0x00000001800014c9	0x000014c9	324 (0x144)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_gear_joint	0x000000018000146a	0x0000146a	325 (0x145)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_joint	0x00000001800027bb	0x000027bb	326 (0x146)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_motor_joint	0x0000000180001f50	0x00001f50	327 (0x147)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_mouse_joint	0x00000001800029fa	0x000029fa	328 (0x148)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_prismatic_joint	0x0000000180001041	0x00001041	329 (0x149)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_pulley_joint	0x0000000180001177	0x00001177	330 (0x14a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_revolute_joint	0x0000000180002252	0x00002252	331 (0x14b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_rope_joint	0x00000001800026ee	0x000026ee	332 (0x14c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_weld_joint	0x0000000180002766	0x00002766	333 (0x14d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_create_wheel_joint	0x0000000180002216	0x00002216	334 (0x14e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_destroy_body	0x00000001800026e9	0x000026e9	335 (0x14f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_destroy_joint	0x000000018000276b	0x0000276b	336 (0x150)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_draw_debug_data	0x0000000180001bd1	0x00001bd1	337 (0x151)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_drop	0x0000000180001fb4	0x00001fb4	338 (0x152)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_dump	0x00000001800014f6	0x000014f6	339 (0x153)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_allow_sleeping	0x0000000180001a23	0x00001a23	340 (0x154)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_auto_clear_forces	0x00000001800023bf	0x000023bf	341 (0x155)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_body_count	0x000000018000175d	0x0000175d	342 (0x156)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_body_list	0x00000001800025c7	0x000025c7	343 (0x157)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_body_list_const	0x0000000180002888	0x00002888	344 (0x158)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_contact_count	0x0000000180002671	0x00002671	345 (0x159)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_contact_list	0x0000000180001ae6	0x00001ae6	346 (0x15a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_contact_list_const	0x000000018000225c	0x0000225c	347 (0x15b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_contact_manager	0x0000000180002a18	0x00002a18	348 (0x15c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_continuous_physics	0x00000001800016f9	0x000016f9	349 (0x15d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_gravity	0x0000000180002950	0x00002950	350 (0x15e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_joint_count	0x0000000180001235	0x00001235	351 (0x15f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_joint_list	0x0000000180001541	0x00001541	352 (0x160)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_joint_list_const	0x0000000180001bea	0x00001bea	353 (0x161)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_profile	0x0000000180001401	0x00001401	354 (0x162)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_proxy_count	0x00000001800025b3	0x000025b3	355 (0x163)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_sub_stepping	0x0000000180002342	0x00002342	356 (0x164)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_tree_balance	0x00000001800015f5	0x000015f5	357 (0x165)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_tree_height	0x00000001800023b0	0x000023b0	358 (0x166)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_tree_quality	0x0000000180001889	0x00001889	359 (0x167)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_get_warm_starting	0x0000000180001ab9	0x00001ab9	360 (0x168)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_is_locked	0x00000001800020ae	0x000020ae	361 (0x169)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_new	0x00000001800019e7	0x000019e7	362 (0x16a)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_query_aabb	0x0000000180001744	0x00001744	363 (0x16b)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_ray_cast	0x0000000180001122	0x00001122	364 (0x16c)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_allow_sleeping	0x0000000180002b08	0x00002b08	365 (0x16d)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_auto_clear_forces	0x00000001800010c8	0x000010c8	366 (0x16e)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_contact_filter	0x0000000180001d84	0x00001d84	367 (0x16f)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_contact_listener	0x0000000180001019	0x00001019	368 (0x170)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_continuous_physics	0x0000000180001311	0x00001311	369 (0x171)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_debug_draw	0x0000000180001267	0x00001267	370 (0x172)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_destruction_listener	0x000000018000112c	0x0000112c	371 (0x173)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_gravity	0x00000001800010aa	0x000010aa	372 (0x174)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_sub_stepping	0x0000000180001f4b	0x00001f4b	373 (0x175)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_set_warm_starting	0x0000000180001bae	0x00001bae	374 (0x176)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_shift_origin	0x0000000180002a59	0x00002a59	375 (0x177)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// World_step	0x0000000180001929	0x00001929	376 (0x178)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	
// WorldManifold_Initialize	0x0000000180001e83	0x00001e83	320 (0x140)	box2djai.dll	C:\Users\Kuju\Projects\JaiBox2D\bin\win\box2djai.dll	Exported Function	


#scope_file

#if OS == .WINDOWS {
    box2d :: #foreign_library "win/box2d";
}

#import "Math";
#import "Basic";