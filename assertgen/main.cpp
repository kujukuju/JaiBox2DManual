#include <iostream>
#include <fstream>

#include "../box2d/lib.cpp"

int main() {
    std::ofstream assertFile;
    assertFile.open("../../asserts.jai");
    assertFile << "#run {\n";
    assertFile << "    assert(size_of(b2WorldManifold) == " << sizeof(b2WorldManifold) << ", \"b2WorldManifold must be of size " << sizeof(b2WorldManifold) << ".\");\n";
    assertFile << "    assert(size_of(b2Manifold) == " << sizeof(b2Manifold) << ", \"b2Manifold must be of size " << sizeof(b2Manifold) << ".\");\n";
    assertFile << "    assert(size_of(b2Transform) == " << sizeof(b2Transform) << ", \"b2Transform must be of size " << sizeof(b2Transform) << ".\");\n";
    assertFile << "    assert(size_of(b2PointState) == " << sizeof(b2PointState) << ", \"b2PointState must be of size " << sizeof(b2PointState) << ".\");\n";
    assertFile << "    assert(size_of(b2AABB) == " << sizeof(b2AABB) << ", \"b2AABB must be of size " << sizeof(b2AABB) << ".\");\n";
    assertFile << "    assert(size_of(b2RayCastOutput) == " << sizeof(b2RayCastOutput) << ", \"b2RayCastOutput must be of size " << sizeof(b2RayCastOutput) << ".\");\n";
    assertFile << "    assert(size_of(b2RayCastInput) == " << sizeof(b2RayCastInput) << ", \"b2RayCastInput must be of size " << sizeof(b2RayCastInput) << ".\");\n";
    assertFile << "    assert(size_of(b2CircleShape) == " << sizeof(b2CircleShape) << ", \"b2CircleShape must be of size " << sizeof(b2CircleShape) << ".\");\n";
    assertFile << "    assert(size_of(b2PolygonShape) == " << sizeof(b2PolygonShape) << ", \"b2PolygonShape must be of size " << sizeof(b2PolygonShape) << ".\");\n";
    assertFile << "    assert(size_of(b2EdgeShape) == " << sizeof(b2EdgeShape) << ", \"b2EdgeShape must be of size " << sizeof(b2EdgeShape) << ".\");\n";
    assertFile << "    assert(size_of(b2ClipVertex) == " << sizeof(b2ClipVertex) << ", \"b2ClipVertex must be of size " << sizeof(b2ClipVertex) << ".\");\n";
    assertFile << "    assert(size_of(b2Shape) == " << sizeof(b2Shape) << ", \"b2Shape must be of size " << sizeof(b2Shape) << ".\");\n";
    assertFile << "    assert(size_of(Draw) == " << sizeof(Draw) << ", \"Draw must be of size " << sizeof(Draw) << ".\");\n";
    assertFile << "    assert(size_of(DrawPolygonCB) == " << sizeof(DrawPolygonCB) << ", \"DrawPolygonCB must be of size " << sizeof(DrawPolygonCB) << ".\");\n";
    assertFile << "    assert(size_of(DrawSolidPolygonCB) == " << sizeof(DrawSolidPolygonCB) << ", \"DrawSolidPolygonCB must be of size " << sizeof(DrawSolidPolygonCB) << ".\");\n";
    assertFile << "    assert(size_of(DrawCircleCB) == " << sizeof(DrawCircleCB) << ", \"DrawCircleCB must be of size " << sizeof(DrawCircleCB) << ".\");\n";
    assertFile << "    assert(size_of(DrawSolidCircleCB) == " << sizeof(DrawSolidCircleCB) << ", \"DrawSolidCircleCB must be of size " << sizeof(DrawSolidCircleCB) << ".\");\n";
    assertFile << "    assert(size_of(DrawSegmentCB) == " << sizeof(DrawSegmentCB) << ", \"DrawSegmentCB must be of size " << sizeof(DrawSegmentCB) << ".\");\n";
    assertFile << "    assert(size_of(DrawTransformCB) == " << sizeof(DrawTransformCB) << ", \"DrawTransformCB must be of size " << sizeof(DrawTransformCB) << ".\");\n";
    assertFile << "    assert(size_of(DrawPointCB) == " << sizeof(DrawPointCB) << ", \"DrawPointCB must be of size " << sizeof(DrawPointCB) << ".\");\n";
    assertFile << "    assert(size_of(b2Draw) == " << sizeof(b2Draw) << ", \"b2Draw must be of size " << sizeof(b2Draw) << ".\");\n";
    assertFile << "    assert(size_of(b2BodyDef) == " << sizeof(b2BodyDef) << ", \"b2BodyDef must be of size " << sizeof(b2BodyDef) << ".\");\n";
    assertFile << "    assert(size_of(b2Fixture) == " << sizeof(b2Fixture) << ", \"b2Fixture must be of size " << sizeof(b2Fixture) << ".\");\n";
    assertFile << "    assert(size_of(b2Body) == " << sizeof(b2Body) << ", \"b2Body must be of size " << sizeof(b2Body) << ".\");\n";
    assertFile << "    assert(size_of(b2FixtureDef) == " << sizeof(b2FixtureDef) << ", \"b2FixtureDef must be of size " << sizeof(b2FixtureDef) << ".\");\n";
    assertFile << "    assert(size_of(b2MassData) == " << sizeof(b2MassData) << ", \"b2MassData must be of size " << sizeof(b2MassData) << ".\");\n";
    assertFile << "    assert(size_of(b2BodyType) == " << sizeof(b2BodyType) << ", \"b2BodyType must be of size " << sizeof(b2BodyType) << ".\");\n";
    assertFile << "    assert(size_of(b2JointEdge) == " << sizeof(b2JointEdge) << ", \"b2JointEdge must be of size " << sizeof(b2JointEdge) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactEdge) == " << sizeof(b2ContactEdge) << ", \"b2ContactEdge must be of size " << sizeof(b2ContactEdge) << ".\");\n";
    assertFile << "    assert(size_of(b2World) == " << sizeof(b2World) << ", \"b2World must be of size " << sizeof(b2World) << ".\");\n";
    assertFile << "    assert(size_of(b2Filter) == " << sizeof(b2Filter) << ", \"b2Filter must be of size " << sizeof(b2Filter) << ".\");\n";
    assertFile << "    assert(size_of(b2DestructionListener) == " << sizeof(b2DestructionListener) << ", \"b2DestructionListener must be of size " << sizeof(b2DestructionListener) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactFilter) == " << sizeof(b2ContactFilter) << ", \"b2ContactFilter must be of size " << sizeof(b2ContactFilter) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactListener) == " << sizeof(b2ContactListener) << ", \"b2ContactListener must be of size " << sizeof(b2ContactListener) << ".\");\n";
    assertFile << "    assert(size_of(b2Joint) == " << sizeof(b2Joint) << ", \"b2Joint must be of size " << sizeof(b2Joint) << ".\");\n";
    assertFile << "    assert(size_of(b2JointDef) == " << sizeof(b2JointDef) << ", \"b2JointDef must be of size " << sizeof(b2JointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2QueryCallback) == " << sizeof(b2QueryCallback) << ", \"b2QueryCallback must be of size " << sizeof(b2QueryCallback) << ".\");\n";
    assertFile << "    assert(size_of(b2RayCastCallback) == " << sizeof(b2RayCastCallback) << ", \"b2RayCastCallback must be of size " << sizeof(b2RayCastCallback) << ".\");\n";
    assertFile << "    assert(size_of(b2Contact) == " << sizeof(b2Contact) << ", \"b2Contact must be of size " << sizeof(b2Contact) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactManager) == " << sizeof(b2ContactManager) << ", \"b2ContactManager must be of size " << sizeof(b2ContactManager) << ".\");\n";
    assertFile << "    assert(size_of(b2Profile) == " << sizeof(b2Profile) << ", \"b2Profile must be of size " << sizeof(b2Profile) << ".\");\n";
    assertFile << "    assert(size_of(DestructionListener) == " << sizeof(DestructionListener) << ", \"DestructionListener must be of size " << sizeof(DestructionListener) << ".\");\n";
    assertFile << "    assert(size_of(SayGoodbyeJointCB) == " << sizeof(SayGoodbyeJointCB) << ", \"SayGoodbyeJointCB must be of size " << sizeof(SayGoodbyeJointCB) << ".\");\n";
    assertFile << "    assert(size_of(SayGoodbyeFixtureCB) == " << sizeof(SayGoodbyeFixtureCB) << ", \"SayGoodbyeFixtureCB must be of size " << sizeof(SayGoodbyeFixtureCB) << ".\");\n";
    assertFile << "    assert(size_of(ContactFilter) == " << sizeof(ContactFilter) << ", \"ContactFilter must be of size " << sizeof(ContactFilter) << ".\");\n";
    assertFile << "    assert(size_of(ShouldCollideCB) == " << sizeof(ShouldCollideCB) << ", \"ShouldCollideCB must be of size " << sizeof(ShouldCollideCB) << ".\");\n";
    assertFile << "    assert(size_of(ContactListener) == " << sizeof(ContactListener) << ", \"ContactListener must be of size " << sizeof(ContactListener) << ".\");\n";
    assertFile << "    assert(size_of(BeginContactCB) == " << sizeof(BeginContactCB) << ", \"BeginContactCB must be of size " << sizeof(BeginContactCB) << ".\");\n";
    assertFile << "    assert(size_of(EndContactCB) == " << sizeof(EndContactCB) << ", \"EndContactCB must be of size " << sizeof(EndContactCB) << ".\");\n";
    assertFile << "    assert(size_of(PreSolveCB) == " << sizeof(PreSolveCB) << ", \"PreSolveCB must be of size " << sizeof(PreSolveCB) << ".\");\n";
    assertFile << "    assert(size_of(PostSolveCB) == " << sizeof(PostSolveCB) << ", \"PostSolveCB must be of size " << sizeof(PostSolveCB) << ".\");\n";
    assertFile << "    assert(size_of(QueryCallback) == " << sizeof(QueryCallback) << ", \"QueryCallback must be of size " << sizeof(QueryCallback) << ".\");\n";
    assertFile << "    assert(size_of(ReportFixtureQueryCB) == " << sizeof(ReportFixtureQueryCB) << ", \"ReportFixtureQueryCB must be of size " << sizeof(ReportFixtureQueryCB) << ".\");\n";
    assertFile << "    assert(size_of(RayCastCallback) == " << sizeof(RayCastCallback) << ", \"RayCastCallback must be of size " << sizeof(RayCastCallback) << ".\");\n";
    assertFile << "    assert(size_of(ReportFixtureRayCastCB) == " << sizeof(ReportFixtureRayCastCB) << ", \"ReportFixtureRayCastCB must be of size " << sizeof(ReportFixtureRayCastCB) << ".\");\n";
    assertFile << "    assert(size_of(b2ChainShape) == " << sizeof(b2ChainShape) << ", \"b2ChainShape must be of size " << sizeof(b2ChainShape) << ".\");\n";
    assertFile << "    assert(size_of(b2BlockAllocator) == " << sizeof(b2BlockAllocator) << ", \"b2BlockAllocator must be of size " << sizeof(b2BlockAllocator) << ".\");\n";
    assertFile << "    assert(size_of(b2DistanceJointDef) == " << sizeof(b2DistanceJointDef) << ", \"b2DistanceJointDef must be of size " << sizeof(b2DistanceJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2DistanceJoint) == " << sizeof(b2DistanceJoint) << ", \"b2DistanceJoint must be of size " << sizeof(b2DistanceJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2FrictionJointDef) == " << sizeof(b2FrictionJointDef) << ", \"b2FrictionJointDef must be of size " << sizeof(b2FrictionJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2FrictionJoint) == " << sizeof(b2FrictionJoint) << ", \"b2FrictionJoint must be of size " << sizeof(b2FrictionJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2GearJointDef) == " << sizeof(b2GearJointDef) << ", \"b2GearJointDef must be of size " << sizeof(b2GearJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2GearJoint) == " << sizeof(b2GearJoint) << ", \"b2GearJoint must be of size " << sizeof(b2GearJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2JointType) == " << sizeof(b2JointType) << ", \"b2JointType must be of size " << sizeof(b2JointType) << ".\");\n";
    assertFile << "    assert(size_of(b2MotorJointDef) == " << sizeof(b2MotorJointDef) << ", \"b2MotorJointDef must be of size " << sizeof(b2MotorJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2MotorJoint) == " << sizeof(b2MotorJoint) << ", \"b2MotorJoint must be of size " << sizeof(b2MotorJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2MouseJointDef) == " << sizeof(b2MouseJointDef) << ", \"b2MouseJointDef must be of size " << sizeof(b2MouseJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2MouseJoint) == " << sizeof(b2MouseJoint) << ", \"b2MouseJoint must be of size " << sizeof(b2MouseJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2PrismaticJointDef) == " << sizeof(b2PrismaticJointDef) << ", \"b2PrismaticJointDef must be of size " << sizeof(b2PrismaticJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2PrismaticJoint) == " << sizeof(b2PrismaticJoint) << ", \"b2PrismaticJoint must be of size " << sizeof(b2PrismaticJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2PulleyJointDef) == " << sizeof(b2PulleyJointDef) << ", \"b2PulleyJointDef must be of size " << sizeof(b2PulleyJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2PulleyJoint) == " << sizeof(b2PulleyJoint) << ", \"b2PulleyJoint must be of size " << sizeof(b2PulleyJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2RevoluteJointDef) == " << sizeof(b2RevoluteJointDef) << ", \"b2RevoluteJointDef must be of size " << sizeof(b2RevoluteJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2RevoluteJoint) == " << sizeof(b2RevoluteJoint) << ", \"b2RevoluteJoint must be of size " << sizeof(b2RevoluteJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2RopeTuning) == " << sizeof(b2RopeTuning) << ", \"b2RopeTuning must be of size " << sizeof(b2RopeTuning) << ".\");\n";
    assertFile << "    assert(size_of(b2RopeDef) == " << sizeof(b2RopeDef) << ", \"b2RopeDef must be of size " << sizeof(b2RopeDef) << ".\");\n";
    assertFile << "    assert(size_of(b2Rope) == " << sizeof(b2Rope) << ", \"b2Rope must be of size " << sizeof(b2Rope) << ".\");\n";
    assertFile << "    assert(size_of(b2WeldJointDef) == " << sizeof(b2WeldJointDef) << ", \"b2WeldJointDef must be of size " << sizeof(b2WeldJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2WeldJoint) == " << sizeof(b2WeldJoint) << ", \"b2WeldJoint must be of size " << sizeof(b2WeldJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2WheelJointDef) == " << sizeof(b2WheelJointDef) << ", \"b2WheelJointDef must be of size " << sizeof(b2WheelJointDef) << ".\");\n";
    assertFile << "    assert(size_of(b2WheelJoint) == " << sizeof(b2WheelJoint) << ", \"b2WheelJoint must be of size " << sizeof(b2WheelJoint) << ".\");\n";
    assertFile << "    assert(size_of(b2Mat22) == " << sizeof(b2Mat22) << ", \"b2Mat22 must be of size " << sizeof(b2Mat22) << ".\");\n";
    assertFile << "    assert(size_of(b2Mat33) == " << sizeof(b2Mat33) << ", \"b2Mat33 must be of size " << sizeof(b2Mat33) << ".\");\n";
    assertFile << "    assert(size_of(b2Sweep) == " << sizeof(b2Sweep) << ", \"b2Sweep must be of size " << sizeof(b2Sweep) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactImpulse) == " << sizeof(b2ContactImpulse) << ", \"b2ContactImpulse must be of size " << sizeof(b2ContactImpulse) << ".\");\n";
    assertFile << "    assert(size_of(b2BlockAllocator) == " << sizeof(b2BlockAllocator) << ", \"b2BlockAllocator must be of size " << sizeof(b2BlockAllocator) << ".\");\n";
    assertFile << "    assert(size_of(b2FixtureProxy) == " << sizeof(b2FixtureProxy) << ", \"b2FixtureProxy must be of size " << sizeof(b2FixtureProxy) << ".\");\n";
    assertFile << "    assert(size_of(b2StackAllocator) == " << sizeof(b2StackAllocator) << ", \"b2StackAllocator must be of size " << sizeof(b2StackAllocator) << ".\");\n";
    assertFile << "    assert(size_of(b2StackEntry) == " << sizeof(b2StackEntry) << ", \"b2StackEntry must be of size " << sizeof(b2StackEntry) << ".\");\n";
    assertFile << "    assert(size_of(b2JointType) == " << sizeof(b2JointType) << ", \"b2JointType must be of size " << sizeof(b2JointType) << ".\");\n";
    assertFile << "    assert(size_of(b2BroadPhase) == " << sizeof(b2BroadPhase) << ", \"b2BroadPhase must be of size " << sizeof(b2BroadPhase) << ".\");\n";
    assertFile << "    assert(size_of(b2DynamicTree) == " << sizeof(b2DynamicTree) << ", \"b2DynamicTree must be of size " << sizeof(b2DynamicTree) << ".\");\n";
    assertFile << "    assert(size_of(b2Pair) == " << sizeof(b2Pair) << ", \"b2Pair must be of size " << sizeof(b2Pair) << ".\");\n";
    assertFile << "    assert(size_of(b2TreeNode) == " << sizeof(b2TreeNode) << ", \"b2TreeNode must be of size " << sizeof(b2TreeNode) << ".\");\n";
    assertFile << "    assert(size_of(b2ManifoldPoint) == " << sizeof(b2ManifoldPoint) << ", \"b2ManifoldPoint must be of size " << sizeof(b2ManifoldPoint) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactFeature) == " << sizeof(b2ContactFeature) << ", \"b2ContactFeature must be of size " << sizeof(b2ContactFeature) << ".\");\n";
    assertFile << "    assert(size_of(b2Rot) == " << sizeof(b2Rot) << ", \"b2Rot must be of size " << sizeof(b2Rot) << ".\");\n";
    assertFile << "    assert(size_of(b2ContactID) == " << sizeof(b2ContactID) << ", \"b2ContactID must be of size " << sizeof(b2ContactID) << ".\");\n";
    assertFile << "    assert(size_of(b2StretchingModel) == " << sizeof(b2StretchingModel) << ", \"b2StretchingModel must be of size " << sizeof(b2StretchingModel) << ".\");\n";
    assertFile << "    assert(size_of(b2BendingModel) == " << sizeof(b2BendingModel) << ", \"b2BendingModel must be of size " << sizeof(b2BendingModel) << ".\");\n";
    assertFile << "    assert(size_of(b2Color) == " << sizeof(b2Color) << ", \"b2Color must be of size " << sizeof(b2Color) << ".\");\n";
    assertFile << "}\n\n#scope_file\n\n#import \"Basic\";\n";
    assertFile.close();
    return 0;
}
