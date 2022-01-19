main :: () {
    print("running...\n");

    print("shape %\n", size_of(b2Shape));
    print("circle %\n", size_of(b2CircleShape));

    test: b2CircleShape;
    print("test circle %\n", test);

    circle_shape := CircleShape_new();
    print("address %\n", circle_shape);

    print("circle %\n", << circle_shape);
}

#scope_file

#load "../module.jai";

#import "Basic";