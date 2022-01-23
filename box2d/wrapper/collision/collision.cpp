#include "collision/collision.h"

void WorldManifold_initialize(b2WorldManifold* self, const b2Manifold* manifold, const b2Transform* xf_a, float radius_a, const b2Transform* xf_b, float radius_b) {
    self->Initialize(manifold, *xf_a, radius_a, *xf_b, radius_b);
}

void get_point_states(b2PointState* s1, b2PointState* s2, const b2Manifold* m1, const b2Manifold* m2) {
    b2GetPointStates(s1, s2, m1, m2);
}

bool AABB_is_valid(b2AABB* self) {
    return self->IsValid();
}

b2Vec2 AABB_get_center(b2AABB* self) {
    return self->GetCenter();
}

b2Vec2 AABB_get_extents(b2AABB* self) {
    return self->GetExtents();
}

float AABB_get_perimeter(b2AABB* self) {
    return self->GetPerimeter();
}

void AABB_combine(b2AABB* self, const b2AABB* aabb) {
    self->Combine(*aabb);
}

void AABB_combine_two(b2AABB* self, const b2AABB* aabb1, const b2AABB* aabb2) {
    self->Combine(*aabb1, *aabb2);
}

bool AABB_contains(b2AABB* self, const b2AABB* aabb) {
    return self->Contains(*aabb);
}

bool AABB_ray_cast(b2AABB* self, b2RayCastOutput* output, const b2RayCastInput* input) {
    return self->RayCast(output, *input);
}

void collide_circles(b2Manifold* manifold, const b2CircleShape* circleA, const b2Transform* xfA, const b2CircleShape* circleB, const b2Transform* xfB) {
    b2CollideCircles(manifold, circleA, *xfA, circleB, *xfB);
}

void collide_polygon_and_circle(b2Manifold* manifold, const b2PolygonShape* polygonA, const b2Transform* xfA, const b2CircleShape* circleB, const b2Transform* xfB) {
    b2CollidePolygonAndCircle(manifold, polygonA, *xfA, circleB, *xfB);
}

void collide_polygons(b2Manifold* manifold, const b2PolygonShape* polygonA, const b2Transform* xfA, const b2PolygonShape* polygonB, const b2Transform* xfB) {
    b2CollidePolygons(manifold, polygonA, *xfA, polygonB, *xfB);
}

void collide_edge_and_circle(b2Manifold* manifold, const b2EdgeShape* polygonA, const b2Transform* xfA, const b2CircleShape* circleB, const b2Transform* xfB) {
    b2CollideEdgeAndCircle(manifold, polygonA, *xfA, circleB, *xfB);
}

void collide_edge_and_polygon(b2Manifold* manifold, const b2EdgeShape* edgeA, const b2Transform* xfA, const b2PolygonShape* circleB, const b2Transform* xfB) {
    b2CollideEdgeAndPolygon(manifold, edgeA, *xfA, circleB, *xfB);
}

uint32_t clip_segment_to_line(b2ClipVertex* vOut, const b2ClipVertex* vIn, const b2Vec2* normal, float offset, int32_t vertexIndexA) {
    return b2ClipSegmentToLine(vOut, vIn, *normal, offset, vertexIndexA);
}

bool test_overlap(const b2Shape* shapeA, int32_t indexA, const b2Shape* shapeB, int32_t indexB, const b2Transform* xfA, const b2Transform* xfB) {
    return b2TestOverlap(shapeA, indexA, shapeB, indexB, *xfA, *xfB);
}

bool test_overlap_aabb(const b2AABB* a, const b2AABB* b) {
    return b2TestOverlap(*a, *b);
}
