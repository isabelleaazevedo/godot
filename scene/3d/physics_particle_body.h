/*************************************************************************/
/*  physics_particle_body.h                                              */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2018 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2018 Godot Engine contributors (cf. AUTHORS.md)    */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

/**
 * @author AndreaCatania
 */

#ifndef PARTICLE_BODY_H
#define PARTICLE_BODY_H

#include "scene/resources/particle_body_model.h"
#include "scene/resources/primitive_meshes.h"
#include "spatial.h"

class ParticleObject : public Spatial {
	GDCLASS(ParticleObject, Spatial);

protected:
	RID rid;

	static void _bind_methods();

public:
	ParticleObject(RID p_rid);
	virtual ~ParticleObject();

	_FORCE_INLINE_ RID get_rid() { return rid; }
};

class ParticleBody : public ParticleObject {
	GDCLASS(ParticleBody, ParticleObject);

	bool reset_particles_to_base_shape;
	Ref<ParticleBodyModel> particle_body_model;

	uint32_t collision_layer;

	Vector<RID> debug_particle_visual_instances;
	Ref<SphereMesh> debug_particle_mesh;

protected:
	static void _bind_methods();

public:
	ParticleBody();
	virtual ~ParticleBody();

	void set_particle_body_model(Ref<ParticleBodyModel> p_shape);
	Ref<ParticleBodyModel> get_particle_body_model() const;

	void add_particle(const Vector3 &p_local_position, real_t p_mass);
	void remove_particle(int p_particle_index);
	void set_collision_layer(uint32_t p_layer);
	uint32_t get_collision_layer() const;

	void set_collision_layer_bit(int p_bit, bool p_value);
	bool get_collision_layer_bit(int p_bit) const;

protected:
	void _notification(int p_what);
	void resource_changed(const RES &p_res);

	void commands_process_internal(Object *p_cmds);

private:
	void _on_script_changed();
	void initialize_debug_resource();
	void update_debug_visual_instances(ParticleBodyCommands *p_cmds);
	void resize_debug_particle_visual_instance(int new_size);
	void reset_debug_particle_positions();
};

#endif // PARTICLE_BODY_H
