// #load "structs.jai";
// #load "functions.jai";
// #load "asserts.jai";

b2Shape :: struct {
    vtable: *void;
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

#run {
    assert(size_of(b2ShapeType) == 4, tprint("b2ShapeType size must be 4. %", size_of(b2ShapeType)));
}

b2CircleShape :: struct {
    using shape: b2Shape;
    m_p: Vector2;
}

CircleShape_new :: () -> *b2CircleShape #must #foreign box2d;

#scope_file

#import "Basic";
#import "Math";

box2d :: #foreign_library "bin/win/box2djai";
