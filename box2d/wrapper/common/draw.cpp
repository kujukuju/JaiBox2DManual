#include "common/draw.h"

//typedef void (*DrawPolygonCB)(RustObject, const b2Vec2*, int32_t, const b2Color*);
//typedef void (*DrawCircleCB)(RustObject, const b2Vec2*, float, const b2Color*);
//typedef void (*DrawSolidCircleCB)(RustObject, const b2Vec2*, float, const b2Vec2*,
//                                  const b2Color*);
//typedef void (*DrawSegmentCB)(RustObject, const b2Vec2*, const b2Vec2*,
//                              const b2Color*);
//typedef void (*DrawTransformCB)(RustObject, const b2Transform*);

//struct DrawLink: public b2Draw {
//    DrawLink(): b2Draw() {}
//    ~DrawLink() {}
//
//    void DrawPolygon(const b2Vec2* vertices, int32_t count,
//                     const b2Color& color) {
//        draw_polygon(object, vertices, count, &color);
//    }
//
//    void DrawSolidPolygon(const b2Vec2* vertices, int32_t count,
//                          const b2Color& color) {
//        draw_solid_polygon(object, vertices, count, &color);
//    }
//
//    void DrawCircle(const b2Vec2& center, float radius,
//                    const b2Color& color) {
//        draw_circle(object, &center, radius, &color);
//    }
//
//    void DrawSolidCircle(const b2Vec2& center, float radius,
//                         const b2Vec2& axis, const b2Color& color) {
//        draw_solid_circle(object, &center, radius, &axis, &color);
//    }
//
//    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2,
//                     const b2Color& color) {
//        draw_segment(object, &p1, &p2, &color);
//    }
//
//    void DrawTransform(const b2Transform& xf) {
//        draw_transform(object, &xf);
//    }
//
//    RustObject object;
//    DrawPolygonCB draw_polygon;
//    DrawPolygonCB draw_solid_polygon;
//    DrawCircleCB draw_circle;
//    DrawSolidCircleCB draw_solid_circle;
//    DrawSegmentCB draw_segment;
//    DrawTransformCB draw_transform;
//};
//
//DrawLink* DrawLink_alloc() {
//    return new DrawLink();
//}
//
//b2Draw* DrawLink_as_base(DrawLink* self) {
//    return static_cast<b2Draw*>(self);
//}
//
//void DrawLink_drop(DrawLink* self) {
//    delete self;
//}
//
//void DrawLink_bind(DrawLink* self,
//                   RustObject o,
//                   DrawPolygonCB dp,
//                   DrawPolygonCB dsp,
//                   DrawCircleCB dc,
//                   DrawSolidCircleCB dsc,
//                   DrawSegmentCB ds,
//                   DrawTransformCB dt) {
//    self->object = o;
//    self->draw_polygon = dp;
//    self->draw_solid_polygon = dsp;
//    self->draw_circle = dc;
//    self->draw_solid_circle = dsc;
//    self->draw_segment = ds;
//    self->draw_transform = dt;
//}
//
//void DrawLink_set_flags(DrawLink* self, uint32_t flags) {
//    self->SetFlags(flags);
//}
