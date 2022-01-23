typedef void (*DrawPolygonCB)(const b2Vec2*, int32_t, b2Color);
typedef void (*DrawSolidPolygonCB)(const b2Vec2*, int32_t, b2Color);
typedef void (*DrawCircleCB)(b2Vec2, float, b2Color);
typedef void (*DrawSolidCircleCB)(b2Vec2, float, b2Vec2, b2Color);
typedef void (*DrawSegmentCB)(b2Vec2, b2Vec2, b2Color);
typedef void (*DrawTransformCB)(b2Transform);
typedef void (*DrawPointCB)(b2Vec2, float, b2Color);

struct Draw : public b2Draw {
    Draw(DrawPolygonCB drawPolygon, DrawSolidPolygonCB drawSolidPolygon, DrawCircleCB drawCircle, DrawSolidCircleCB drawSolidCircle, DrawSegmentCB drawSegment, DrawTransformCB drawTransform, DrawPointCB drawPoint)
        : m_drawPolygon(*drawPolygon),
          m_drawSolidPolygon(*drawSolidPolygon),
          m_drawCircle(*drawCircle),
          m_drawSolidCircle(*drawSolidCircle),
          m_drawSegment(*drawSegment),
          m_drawTransform(*drawTransform),
          m_drawPoint(*drawPoint) {}

    ~Draw() {}

    void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override {
        m_drawPolygon(vertices, vertexCount, color);
    }

    void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override {
        m_drawSolidPolygon(vertices, vertexCount, color);
    }

    void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) override {
        m_drawCircle(center, radius, color);
    }

    void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) override {
        m_drawSolidCircle(center, radius, axis, color);
    }

    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override {
        m_drawSegment(p1, p2, color);
    }

    void DrawTransform(const b2Transform& xf) override {
        m_drawTransform(xf);
    }

    void DrawPoint(const b2Vec2& p, float size, const b2Color& color) override {
        m_drawPoint(p, size, color);
    }

    DrawPolygonCB m_drawPolygon;
    DrawSolidPolygonCB m_drawSolidPolygon;
    DrawCircleCB m_drawCircle;
    DrawSolidCircleCB m_drawSolidCircle;
    DrawSegmentCB m_drawSegment;
    DrawTransformCB m_drawTransform;
    DrawPointCB m_drawPoint;
};

EXPORT Draw* Draw_new(DrawPolygonCB drawPolygon, DrawSolidPolygonCB drawSolidPolygon, DrawCircleCB drawCircle, DrawSolidCircleCB drawSolidCircle, DrawSegmentCB drawSegment, DrawTransformCB drawTransform, DrawPointCB drawPoint);
EXPORT Draw Draw_create(DrawPolygonCB drawPolygon, DrawSolidPolygonCB drawSolidPolygon, DrawCircleCB drawCircle, DrawSolidCircleCB drawSolidCircle, DrawSegmentCB drawSegment, DrawTransformCB drawTransform, DrawPointCB drawPoint);
EXPORT void Draw_set_flags(b2Draw* self, uint32_t flags);
EXPORT uint32_t Draw_get_flags(b2Draw* self);
EXPORT void Draw_append_flags(b2Draw* self, uint32_t flags);
EXPORT void Draw_clear_flags(b2Draw* self, uint32_t flags);