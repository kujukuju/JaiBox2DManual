#include <Box2D/Box2D.h>
#include <stdint.h>

#define EXPORT __declspec(dllexport)

extern "C" {

typedef int8_t i8;
typedef int16_t i16;
typedef int32_t i32;
typedef int64_t i64;
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef float f32;
typedef double f64;
typedef void* RustObject;
struct RustFatObject {
   void* raw1;
   void* raw2;
};

#include "wrapper/common/draw.cpp"

#include "wrapper/dynamics/body.cpp"
#include "wrapper/dynamics/fixture.cpp"
#include "wrapper/dynamics/world.cpp"
#include "wrapper/dynamics/world_callbacks.cpp"

#include "wrapper/collision/collision.cpp"
#include "wrapper/collision/shapes/shape.cpp"
#include "wrapper/collision/shapes/chain_shape.cpp"
#include "wrapper/collision/shapes/circle_shape.cpp"
#include "wrapper/collision/shapes/edge_shape.cpp"
#include "wrapper/collision/shapes/polygon_shape.cpp"

#include "wrapper/dynamics/joints/joint.cpp"
#include "wrapper/dynamics/joints/distance_joint.cpp"
#include "wrapper/dynamics/joints/friction_joint.cpp"
#include "wrapper/dynamics/joints/gear_joint.cpp"
#include "wrapper/dynamics/joints/motor_joint.cpp"
#include "wrapper/dynamics/joints/mouse_joint.cpp"
#include "wrapper/dynamics/joints/prismatic_joint.cpp"
#include "wrapper/dynamics/joints/pulley_joint.cpp"
#include "wrapper/dynamics/joints/revolute_joint.cpp"
#include "wrapper/dynamics/joints/rope_joint.cpp"
#include "wrapper/dynamics/joints/weld_joint.cpp"
#include "wrapper/dynamics/joints/wheel_joint.cpp"

}
