## Jai Box2D

## Deprecated-ish

This library is the most up to date version of box2d (as of august 2022), and I took the most up to date version of googles liquidfun library build on 8 year old box2d, and merged it in with the latest version of box2d.

If you need liquidfun, this library is for you.

However, I've had a few undefined bugs when doing some specific things with this library. I think if I auto generate the bindings based on the C compatability layer it would fix everything, but I personally no longer need liquidfun, so instead I generated this:

https://github.com/kujukuju/JaiBox2DAuto

And if you want a more simple version of box2d (that's just a wrapper on top of the above library) check out:

https://github.com/kujukuju/KodaBoxJai

## Notes

I might update this in the future.

My potential planned changes are:

1. Remove a lot of the unnecessary pointers for most functions.
2. Possibly change all the binding syntax to be more cpp style.

### Example

```jai
world: b2World = World_create(*Vector2.{0, 0});

body_def: b2BodyDef = BodyDef_create();
body_def.type = b2BodyType.b2_kinematicBody;
body_def.position = Vector2.{0.0, 0.0};
body_def.fixedRotation = true;

shape: b2PolygonShape = PolygonShape_create();
PolygonShape_set_as_box(*shape, 0.1, 0.2);

fixture_def: b2FixtureDef = FixtureDef_create();
fixture_def.shape = *shape;
fixture_def.friction = 0;
fixture_def.restitution = 0;

body: *b2Body = World_create_body(*world, *body_def);
Body_create_fixture(body, *fixture_def);

// ...

World_step(*world, 0.01667, 8, 3);
```


### Note

You can update b2_lengthUnitsPerMEter in b2_settings.h to change the scale and run `jai generate.jai` to rebuild the dll. But I'm not sure how this will work with liquidfun since this didn't exist in box2d 1.3.0.