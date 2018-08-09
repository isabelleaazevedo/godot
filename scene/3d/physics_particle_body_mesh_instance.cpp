/*************************************************************************/
/*  physics_particle_body_mesh_instance.h                                */
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

#include "physics_particle_body_mesh_instance.h"

#include "engine.h"
#include "scene/3d/physics_particle_body.h"
#include "scene/3d/skeleton.h"

void ParticleBodyMeshInstance::_bind_methods() {
}

void ParticleBodyMeshInstance::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_ENTER_TREE: {

			ERR_FAIL_COND(particle_body);

			if (!Engine::get_singleton()->is_editor_hint()) {

				set_as_toplevel(true);
				set_global_transform(Transform());
			}

			particle_body = Object::cast_to<ParticleBody>(get_parent());
			if (particle_body) {
				particle_body->set_particle_body_mesh(this);
				prepare_mesh_skeleton_deformation();
			}

		} break;
		case NOTIFICATION_LOCAL_TRANSFORM_CHANGED: {

			if (!Engine::get_singleton()->is_editor_hint())
				return;

			if (!particle_body)
				return;
			particle_body->set_global_transform(get_global_transform());
			set_notify_local_transform(false);
			set_transform(Transform());
			set_notify_local_transform(true);
		} break;
		case NOTIFICATION_EXIT_TREE: {
			if (particle_body) {
				particle_body->set_particle_body_mesh(NULL);
			}
		} break;
	}
}

ParticleBodyMeshInstance::ParticleBodyMeshInstance() :
		MeshInstance(),
		particle_body(NULL),
		skeleton(NULL),
		rendering_approach(RENDERING_UPDATE_APPROACH_NONE) {

	if (Engine::get_singleton()->is_editor_hint())
		set_notify_local_transform(true);
}

void ParticleBodyMeshInstance::update_mesh(ParticleBodyCommands *p_cmds) {
	switch (rendering_approach) {
		case RENDERING_UPDATE_APPROACH_PVP:
			update_mesh_pvparticles(p_cmds);
			break;
		case RENDERING_UPDATE_APPROACH_SKELETON:
			update_mesh_skeleton(p_cmds);
			break;
	}
}

void ParticleBodyMeshInstance::update_mesh_pvparticles(ParticleBodyCommands *p_cmds) {
}

void ParticleBodyMeshInstance::update_mesh_skeleton(ParticleBodyCommands *p_cmds) {
	const int rigids_count = ParticlePhysicsServer::get_singleton()->body_get_rigid_count(particle_body->get_rid());
	const PoolVector<Vector3>::Read rigids_local_pos_r = particle_body->get_particle_body_model()->get_clusters_positions().read();

	for (int i = 0; i < rigids_count; ++i) {

		Transform t(Basis(p_cmds->get_rigid_rotation(i)), p_cmds->get_rigid_position(i));
		t.translate(rigids_local_pos_r[i] * -1);
		skeleton->set_bone_pose(i, t);
	}
}

void ParticleBodyMeshInstance::prepare_mesh_for_rendering() {

	if (Engine::get_singleton()->is_editor_hint())
		return;

	Ref<ParticleBodyModel> model = particle_body->get_particle_body_model();
	if (model.is_null())
		return;

	if (model->get_clusters_positions().size())
		prepare_mesh_skeleton_deformation();

	else if (model->get_dynamic_triangles_indices().size())
		prepare_mesh_for_pvparticles();
}

void ParticleBodyMeshInstance::prepare_mesh_for_pvparticles() {

	Ref<ParticleBodyModel> model = particle_body->get_particle_body_model();
	if (model.is_null())
		return;

	ERR_FAIL_COND(!model->get_dynamic_triangles_indices().size());

	ERR_FAIL_COND(!get_mesh()->get_surface_count());

	// Get current mesh array and create new mesh array with necessary flag for softbody
	Array surface_arrays = get_mesh()->surface_get_arrays(0);
	Array surface_blend_arrays = get_mesh()->surface_get_blend_shape_arrays(0);
	uint32_t surface_format = get_mesh()->surface_get_format(0);

	surface_format &= ~(Mesh::ARRAY_COMPRESS_VERTEX | Mesh::ARRAY_COMPRESS_NORMAL);
	surface_format |= Mesh::ARRAY_FLAG_USE_DYNAMIC_UPDATE;

	Ref<ArrayMesh> soft_mesh;
	soft_mesh.instance();
	soft_mesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, surface_arrays, surface_blend_arrays, surface_format);

	set_mesh(soft_mesh);
	rendering_approach = RENDERING_UPDATE_APPROACH_PVP;
}

void ParticleBodyMeshInstance::prepare_mesh_skeleton_deformation() {

	if (Engine::get_singleton()->is_editor_hint())
		return;

	if (get_mesh().is_null())
		return;

	Ref<ParticleBodyModel> model = particle_body->get_particle_body_model();
	if (model.is_null())
		return;

	ERR_FAIL_COND(!model->get_clusters_positions().size());
	ERR_FAIL_COND(get_mesh()->get_surface_count() != 1);

	const int surface_id = 0;

	PoolVector<Vector3>::Read clusters_pos_r = model->get_clusters_positions().read();
	const int bone_count(model->get_clusters_positions().size());

	Array array_mesh = get_mesh()->surface_get_arrays(surface_id).duplicate();
	PoolVector<Vector3> vertices = array_mesh[VS::ARRAY_VERTEX];
	PoolVector<Vector3>::Read vertices_read = vertices.read();
	const int vertex_count(vertices.size());

	PoolVector<float> weights; // The weight value for vertex
	PoolVector<int> bone_indices; // The index of bone relative of vertex
	int max_weights_per_vertex = 0;

	ParticlePhysicsServer::get_singleton()->create_skeleton(clusters_pos_r.ptr(), bone_count, vertices_read.ptr(), vertex_count, &weights, &bone_indices, &max_weights_per_vertex);

	ERR_FAIL_COND(max_weights_per_vertex != VS::ARRAY_WEIGHTS_SIZE);

	if (!skeleton) {
		skeleton = memnew(Skeleton);
		add_child(skeleton);
		set_skeleton_path(skeleton->get_path());
	} else {
		skeleton->clear_bones();
	}

	for (int i(0); i < bone_count; ++i) {
		skeleton->add_bone("particle_" + String::num(i));
		skeleton->set_bone_disable_rest(i, true);
	}

	// Create skeleton using these info:
	array_mesh[VS::ARRAY_WEIGHTS] = weights;
	array_mesh[VS::ARRAY_BONES] = bone_indices;

	Ref<ArrayMesh> new_mesh;
	new_mesh.instance();
	new_mesh->add_surface_from_arrays(get_mesh()->surface_get_primitive_type(surface_id), array_mesh);

	set_mesh(new_mesh);

	rendering_approach = RENDERING_UPDATE_APPROACH_SKELETON;
}
