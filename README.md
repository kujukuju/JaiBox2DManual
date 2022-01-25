## Jai Box2D

I might update this significantly in the near future.

My potential planned changes are:

1. Remove a lot of the unnecessary pointers for most functions.
2. Update the Box2D version to the latest to get all bug fixes within the past 8 years.
3. Take the google liquidfun plugin that's based on the 8 year old version of box2d (this) and smash it into the updated version of box2d.
4. Possibly change all the binding syntax to be more cpp style.

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