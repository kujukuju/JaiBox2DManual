EXPORT b2World* World_new(const b2Vec2* gravity);
EXPORT void World_drop(b2World* self);
EXPORT b2World World_create(const b2Vec2 gravity);
EXPORT void World_set_destruction_listener(b2World* self,
                                    b2DestructionListener* listener);
EXPORT void World_set_contact_filter(b2World* self, b2ContactFilter* filter);
EXPORT void World_set_contact_listener(b2World* self,
                                b2ContactListener* listener);
EXPORT void World_set_debug_draw(b2World* self, b2Draw* draw);
EXPORT b2Body* World_create_body(b2World* self, const b2BodyDef* def);
EXPORT void World_destroy_body(b2World* self, b2Body* body);
EXPORT b2Joint* World_create_joint(b2World* self, const b2JointDef* def);
EXPORT void World_destroy_joint(b2World* self, b2Joint* joint);
EXPORT void World_step(b2World* self,
                float time_step,
                int32_t velocity_iterations,
                int32_t position_iterations);
EXPORT void World_clear_forces(b2World* self);
EXPORT void World_draw_debug_data(b2World* self);
EXPORT void World_query_aabb(const b2World* self,
                      b2QueryCallback* callback,
                      const b2AABB* aabb);
EXPORT void World_ray_cast(const b2World* self,
                    b2RayCastCallback* callback,
                    const b2Vec2* p1, const b2Vec2* p2);
EXPORT b2Body* World_get_body_list(b2World* self);
EXPORT b2Joint* World_get_joint_list(b2World* self);
EXPORT b2Contact* World_get_contact_list(b2World* self);
EXPORT void World_set_allow_sleeping(b2World* self, bool flag);
EXPORT bool World_get_allow_sleeping(const b2World* self);
EXPORT void World_set_warm_starting(b2World* self, bool flag);
EXPORT bool World_get_warm_starting(const b2World* self);
EXPORT void World_set_continuous_physics(b2World* self, bool flag);
EXPORT bool World_get_continuous_physics(const b2World* self);
EXPORT void World_set_sub_stepping(b2World* self, bool flag);
EXPORT bool World_get_sub_stepping(const b2World* self);
EXPORT int32_t World_get_proxy_count(const b2World* self);
EXPORT int32_t World_get_body_count(const b2World* self);
EXPORT int32_t World_get_joint_count(const b2World* self);
EXPORT int32_t World_get_contact_count(const b2World* self);
EXPORT int32_t World_get_tree_height(const b2World* self);
EXPORT int32_t World_get_tree_balance(const b2World* self);
EXPORT float World_get_tree_quality(const b2World* self);
EXPORT void World_set_gravity(b2World* self, const b2Vec2* gravity);
EXPORT b2Vec2 World_get_gravity(const b2World* self);
EXPORT bool World_is_locked(const b2World* self);
EXPORT void World_set_auto_clear_forces(b2World* self, bool flag);
EXPORT bool World_get_auto_clear_forces(const b2World* self);
EXPORT void World_shift_origin(b2World* self, const b2Vec2* origin);
EXPORT const b2ContactManager* World_get_contact_manager(const b2World* self);
EXPORT const b2Profile* World_get_profile(const b2World* self);
EXPORT void World_dump(b2World* self);