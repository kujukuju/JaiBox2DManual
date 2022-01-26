EXPORT b2ParticleGroupDef* ParticleGroupDef_new();
EXPORT b2ParticleGroupDef ParticleGroupDef_create();
EXPORT b2ParticleGroup* ParticleGroup_get_next(b2ParticleGroup* self);
EXPORT b2ParticleSystem* ParticleGroup_get_particle_system(b2ParticleGroup* self);
EXPORT int32 ParticleGroup_get_particle_count(b2ParticleGroup* self);
EXPORT int32 ParticleGroup_get_buffer_index(b2ParticleGroup* self);
EXPORT bool ParticleGroup_contains_particle(b2ParticleGroup* self, int32 index);
EXPORT uint32 ParticleGroup_get_all_particle_flags(b2ParticleGroup* self);
EXPORT uint32 ParticleGroup_get_group_flags(b2ParticleGroup* self);
EXPORT void ParticleGroup_set_group_flags(b2ParticleGroup* self, uint32 flags);
EXPORT float32 ParticleGroup_get_mass(b2ParticleGroup* self);
EXPORT float32 ParticleGroup_get_inertia(b2ParticleGroup* self);
EXPORT b2Vec2 ParticleGroup_get_center(b2ParticleGroup* self);
EXPORT b2Vec2 ParticleGroup_get_linear_velocity(b2ParticleGroup* self);
EXPORT float32 ParticleGroup_get_angular_velocity(b2ParticleGroup* self);
EXPORT b2Transform ParticleGroup_get_transform(b2ParticleGroup* self);
EXPORT b2Vec2 ParticleGroup_get_position(b2ParticleGroup* self);
EXPORT float32 ParticleGroup_get_angle(b2ParticleGroup* self);
EXPORT b2Vec2 ParticleGroup_get_linear_velocity_from_world_point(b2ParticleGroup* self, const b2Vec2* worldPoint);
EXPORT void* ParticleGroup_get_user_data(b2ParticleGroup* self);
EXPORT void ParticleGroup_set_user_data(b2ParticleGroup* self, void* data);
EXPORT void ParticleGroup_apply_force(b2ParticleGroup* self, const b2Vec2* force);
EXPORT void ParticleGroup_apply_linear_impulse(b2ParticleGroup* self, const b2Vec2* impulse);
EXPORT void ParticleGroup_destroy_particles(b2ParticleGroup* self, bool callDestructionListener);