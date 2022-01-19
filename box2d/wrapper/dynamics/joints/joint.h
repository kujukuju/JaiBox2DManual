EXPORT int32_t Joint_get_type(const b2Joint* self);
EXPORT b2Body* Joint_get_body_a(b2Joint* self);
EXPORT b2Body* Joint_get_body_b(b2Joint* self);
EXPORT b2Vec2 Joint_get_anchor_a_virtual(const b2Joint* self);
EXPORT b2Vec2 Joint_get_anchor_b_virtual(const b2Joint* self);
EXPORT b2Vec2 Joint_get_reaction_force_virtual(const b2Joint* self,
                                        float inv_dt);
EXPORT float Joint_get_reaction_torque_virtual(const b2Joint* self,
                                      float inv_dt);
EXPORT b2Joint* Joint_get_next(b2Joint* self);
EXPORT void* Joint_get_user_data(const b2Joint* self);
EXPORT void Joint_set_user_data(b2Joint* self, void* data);
EXPORT bool Joint_is_active(const b2Joint* self);
EXPORT bool Joint_get_collide_connected(const b2Joint* self);
EXPORT void Joint_dump_virtual(b2Joint* self);
EXPORT void Joint_shift_origin_virtual(b2Joint* self, const b2Vec2* origin);