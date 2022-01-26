#include "common/draw.h"

Draw* Draw_new(DrawPolygonCB drawPolygon, DrawSolidPolygonCB drawSolidPolygon, DrawCircleCB drawCircle, DrawSolidCircleCB drawSolidCircle, DrawParticlesCB drawParticles, DrawSegmentCB drawSegment, DrawTransformCB drawTransform, DrawPointCB drawPoint) {
    return new Draw(drawPolygon, drawSolidPolygon, drawCircle, drawSolidCircle, drawParticles, drawSegment, drawTransform, drawPoint);
}

Draw Draw_create(DrawPolygonCB drawPolygon, DrawSolidPolygonCB drawSolidPolygon, DrawCircleCB drawCircle, DrawSolidCircleCB drawSolidCircle, DrawParticlesCB drawParticles, DrawSegmentCB drawSegment, DrawTransformCB drawTransform, DrawPointCB drawPoint) {
    return Draw(drawPolygon, drawSolidPolygon, drawCircle, drawSolidCircle, drawParticles, drawSegment, drawTransform, drawPoint);
}

void Draw_set_flags(b2Draw* self, uint32_t flags) {
    self->SetFlags(flags);
}

uint32_t Draw_get_flags(b2Draw* self) {
    return self->GetFlags();
}

void Draw_append_flags(b2Draw* self, uint32_t flags) {
    self->AppendFlags(flags);
}

void Draw_clear_flags(b2Draw* self, uint32_t flags) {
    self->ClearFlags(flags);
}
