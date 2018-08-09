/*************************************************************************/
/*  physics_particle_body_editor_plugin.h                                */
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

#include "physics_particle_body_editor_plugin.h"

#include "editor/spatial_editor_gizmos.h"
#include "scene/gui/box_container.h"
#include "scene/gui/check_box.h"
#include "servers/particle_physics_server.h"
#include "spatial_editor_plugin.h"

void ParticleBodyEditor::_menu_option(int p_option) {
	switch (p_option) {
		case MENU_OPTION_CREATE_PARTICLE_SOFT_BODY:
			soft_body_dialog.dialog->popup_centered(Vector2(200, 700));
			break;
		case MENU_OPTION_CREATE_PARTICLE_RIGID_BODY:
			rigid_body_dialog.dialog->popup_centered(Vector2(200, 120));
			break;
		case MENU_OPTION_CREATE_PARTICLE_CLOTH:
			cloth_dialog.dialog->popup_centered(Vector2(200, 300));
			break;
	}
}

void ParticleBodyEditor::_create_soft_body() {

	ERR_FAIL_COND(!node);
	ERR_FAIL_COND(!node->get_particle_body_mesh());
	ERR_FAIL_COND(node->get_particle_body_mesh()->get_mesh().is_null());

	Ref<ParticleBodyModel> model = ParticlePhysicsServer::get_singleton()->create_soft_particle_body_model(
			node->get_particle_body_mesh()->get_mesh()->generate_triangle_mesh(),
			soft_body_dialog.radius_input->get_value(),
			soft_body_dialog.global_stiffness_input->get_value(),
			soft_body_dialog.internal_sample_check->is_pressed(),
			soft_body_dialog.particle_spacing_input->get_value(),
			soft_body_dialog.sampling_input->get_value(),
			soft_body_dialog.clusterSpacing_input->get_value(),
			soft_body_dialog.clusterRadius_input->get_value(),
			soft_body_dialog.clusterStiffness_input->get_value(),
			soft_body_dialog.linkRadius_input->get_value(),
			soft_body_dialog.linkStiffness_input->get_value(),
			soft_body_dialog.plastic_threshold_input->get_value(),
			soft_body_dialog.plastic_creep_input->get_value());

	UndoRedo *ur = EditorNode::get_singleton()->get_undo_redo();

	ur->create_action(TTR("Create particle Soft body"));
	ur->add_do_method(node, "set_particle_body_model", model);
	ur->add_undo_method(node, "set_particle_body_model", node->get_particle_body_model());
	ur->commit_action();
}

void ParticleBodyEditor::_create_rigid_body() {

	ERR_FAIL_COND(!node);
	ERR_FAIL_COND(!node->get_particle_body_mesh());
	ERR_FAIL_COND(node->get_particle_body_mesh()->get_mesh().is_null());

	Ref<ParticleBodyModel> model = ParticlePhysicsServer::get_singleton()->create_rigid_particle_body_model(
			node->get_particle_body_mesh()->get_mesh()->generate_triangle_mesh(),
			rigid_body_dialog.radius_input->get_value(),
			rigid_body_dialog.expand_input->get_value());

	UndoRedo *ur = EditorNode::get_singleton()->get_undo_redo();

	ur->create_action(TTR("Create particle Rigid body"));
	ur->add_do_method(node, "set_particle_body_model", model);
	ur->add_undo_method(node, "set_particle_body_model", node->get_particle_body_model());
	ur->commit_action();
}

void ParticleBodyEditor::_create_cloth() {

	ERR_FAIL_COND(!node);
	ERR_FAIL_COND(!node->get_particle_body_mesh());
	ERR_FAIL_COND(node->get_particle_body_mesh()->get_mesh().is_null());

	Ref<ParticleBodyModel> model = ParticlePhysicsServer::get_singleton()->create_cloth_particle_body_model(
			node->get_particle_body_mesh()->get_mesh()->generate_triangle_mesh(),
			cloth_dialog.stretch_stiffness_input->get_value(),
			cloth_dialog.bend_stiffness_input->get_value(),
			cloth_dialog.tether_stiffness_input->get_value(),
			cloth_dialog.tether_give_input->get_value(),
			cloth_dialog.pressure_input->get_value());

	UndoRedo *ur = EditorNode::get_singleton()->get_undo_redo();

	ur->create_action(TTR("Create cloth"));
	ur->add_do_method(node, "set_particle_body_model", model);
	ur->add_undo_method(node, "set_particle_body_model", node->get_particle_body_model());
	ur->commit_action();
}

void ParticleBodyEditor::_toggle_show_hide_gizmo() {
	if (!node)
		return;

	node->draw_gizmo = show_gizmo_btn->is_pressed();
	node->update_gizmo();
	if (node->draw_gizmo)
		show();
	else
		hide();
}

void ParticleBodyEditor::_mass_changed(real_t p_mass) {
	if (!node)
		return;

	Ref<ParticleBodyModel> model = node->get_particle_body_model();
	if (model.is_null())
		return;

	Ref<ParticleBodySpatialGizmo> gizmo = node->get_gizmo();
	if (gizmo.is_null())
		return;

	PoolVector<real_t>::Write masses = model->get_masses_ref().write();
	for (int i(0); i < gizmo->get_selected_particles().size(); ++i) {

		masses[gizmo->get_selected_particles()[i]] = p_mass;
	}

	node->update_gizmo();
}

void ParticleBodyEditor::_node_removed(Node *p_node) {

	if (p_node == node) {
		node = NULL;
		options->hide();
	}
}

void ParticleBodyEditor::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_menu_option"), &ParticleBodyEditor::_menu_option);
	ClassDB::bind_method(D_METHOD("_create_soft_body"), &ParticleBodyEditor::_create_soft_body);
	ClassDB::bind_method(D_METHOD("_create_rigid_body"), &ParticleBodyEditor::_create_rigid_body);
	ClassDB::bind_method(D_METHOD("_create_cloth"), &ParticleBodyEditor::_create_cloth);
	ClassDB::bind_method(D_METHOD("_node_removed"), &ParticleBodyEditor::_node_removed);
	ClassDB::bind_method(D_METHOD("_toggle_show_hide_gizmo"), &ParticleBodyEditor::_toggle_show_hide_gizmo);
	ClassDB::bind_method(D_METHOD("_mass_changed", "mass"), &ParticleBodyEditor::_mass_changed);
}

void make_spin_box(SpinBox *&r_spinbox, float p_min, float p_max, float p_step, float p_value, VBoxContainer *dialog_vbc, const String &p_label) {
	r_spinbox = memnew(SpinBox);
	r_spinbox->set_min(p_min);
	r_spinbox->set_max(p_max);
	r_spinbox->set_step(p_step);
	r_spinbox->set_value(p_value);
	dialog_vbc->add_margin_child(p_label, r_spinbox);
}

void make_check_box(CheckBox *&r_check, bool p_pressed, VBoxContainer *dialog_vbc, const String &p_label) {
	r_check = memnew(CheckBox);
	r_check->set_pressed(p_pressed);
	dialog_vbc->add_margin_child(p_label, r_check);
}

ParticleBodyEditor::ParticleBodyEditor() {

	options = memnew(MenuButton);
	SpatialEditor::get_singleton()->add_control_to_menu_panel(options);

	options->set_text(TTR("Particle Body"));
	options->set_icon(EditorNode::get_singleton()->get_gui_base()->get_icon("ParticleBody", "EditorIcons"));

	options->get_popup()->add_item(TTR("Create particle Soft body"), MENU_OPTION_CREATE_PARTICLE_SOFT_BODY);
	options->get_popup()->add_item(TTR("Create particle Rigid body"), MENU_OPTION_CREATE_PARTICLE_RIGID_BODY);
	options->get_popup()->add_item(TTR("Create particle Cloth"), MENU_OPTION_CREATE_PARTICLE_CLOTH);

	options->get_popup()->connect("id_pressed", this, "_menu_option");

	// Soft body creation dialog
	{

		soft_body_dialog.dialog = memnew(ConfirmationDialog);
		soft_body_dialog.dialog->set_title(TTR("Create particle Soft body"));
		soft_body_dialog.dialog->get_ok()->set_text(TTR("Create"));

		VBoxContainer *dialog_vbc = memnew(VBoxContainer);
		soft_body_dialog.dialog->add_child(dialog_vbc);

		make_spin_box(soft_body_dialog.radius_input, 0.001, 1, 0.001, 0.1, dialog_vbc, TTR("Radius:"));
		make_spin_box(soft_body_dialog.global_stiffness_input, 0, 1, 0.001, 0.1, dialog_vbc, TTR("Global stiffness:"));
		make_check_box(soft_body_dialog.internal_sample_check, true, dialog_vbc, TTR("Internal sampling:"));
		make_spin_box(soft_body_dialog.particle_spacing_input, 0.001, 10, 0.01, 1, dialog_vbc, TTR("Particle spacing:"));
		make_spin_box(soft_body_dialog.sampling_input, 0.001, 1, 0.01, 1, dialog_vbc, TTR("Sampling:"));
		make_spin_box(soft_body_dialog.clusterSpacing_input, 0.001, 100, 0.1, 1, dialog_vbc, TTR("Cluster spacing:"));
		make_spin_box(soft_body_dialog.clusterRadius_input, 0.1, 100, 0.1, 2, dialog_vbc, TTR("Cluster radius:"));
		make_spin_box(soft_body_dialog.clusterStiffness_input, 0.01, 1, 0.01, 0.2, dialog_vbc, TTR("Cluster stiffness:"));
		make_spin_box(soft_body_dialog.linkRadius_input, 0.01, 50, 0.01, 0.5, dialog_vbc, TTR("Link radius:"));
		make_spin_box(soft_body_dialog.linkStiffness_input, 0.01, 1, 0.01, 0.1, dialog_vbc, TTR("Link stiffness:"));
		make_spin_box(soft_body_dialog.plastic_threshold_input, 0, 10, 0.01, 0, dialog_vbc, TTR("Plastic threshold:"));
		make_spin_box(soft_body_dialog.plastic_creep_input, 0, 10, 0.01, 0, dialog_vbc, TTR("Plastic creep:"));

		add_child(soft_body_dialog.dialog);
		soft_body_dialog.dialog->connect("confirmed", this, "_create_soft_body");
	}

	// Rigid body creation dialog
	{
		rigid_body_dialog.dialog = memnew(ConfirmationDialog);
		rigid_body_dialog.dialog->set_title(TTR("Create particle Rigid body"));
		rigid_body_dialog.dialog->get_ok()->set_text(TTR("Create"));

		VBoxContainer *dialog_vbc = memnew(VBoxContainer);
		rigid_body_dialog.dialog->add_child(dialog_vbc);

		make_spin_box(rigid_body_dialog.radius_input, 0.001, 1, 0.001, 0.1, dialog_vbc, TTR("Radius:"));
		make_spin_box(rigid_body_dialog.expand_input, 0.0, 10, 0.001, 0.05, dialog_vbc, TTR("Expand:"));

		add_child(rigid_body_dialog.dialog);
		rigid_body_dialog.dialog->connect("confirmed", this, "_create_rigid_body");
	}

	// Cloth body creation dialog
	{
		cloth_dialog.dialog = memnew(ConfirmationDialog);
		cloth_dialog.dialog->set_title(TTR("Create particle Cloth"));
		cloth_dialog.dialog->get_ok()->set_text(TTR("Create"));

		VBoxContainer *dialog_vbc = memnew(VBoxContainer);
		cloth_dialog.dialog->add_child(dialog_vbc);

		make_spin_box(cloth_dialog.stretch_stiffness_input, 0.01, 1, 0.01, 0.8, dialog_vbc, TTR("Stretch stiffness:"));
		make_spin_box(cloth_dialog.bend_stiffness_input, 0.01, 1, 0.01, 0.8, dialog_vbc, TTR("Bend stiffness:"));
		make_spin_box(cloth_dialog.tether_stiffness_input, 0.01, 1, 0.01, 0.8, dialog_vbc, TTR("Tether stiffness:"));
		make_spin_box(cloth_dialog.tether_give_input, 0.01, 1, 0.01, 0.8, dialog_vbc, TTR("Tether give:"));
		make_spin_box(cloth_dialog.pressure_input, 0.0, 10, 0.001, 1, dialog_vbc, TTR("Pressure:"));

		add_child(cloth_dialog.dialog);
		cloth_dialog.dialog->connect("confirmed", this, "_create_cloth");
	}

	set_custom_minimum_size(Size2(200, 0) * EDSCALE);
	show_gizmo_btn = memnew(Button);
	SpatialEditor::get_singleton()->add_control_to_menu_panel(show_gizmo_btn);
	show_gizmo_btn->set_text(TTR("Show gizmo"));
	show_gizmo_btn->set_toggle_mode(true);
	show_gizmo_btn->connect("pressed", this, "_toggle_show_hide_gizmo");

	// Particle inspector
	VBoxContainer *inspector_vb = memnew(VBoxContainer);
	inspector_vb->set_h_size_flags(SIZE_EXPAND_FILL);
	inspector_vb->set_custom_minimum_size(Size2(100, 0) * EDSCALE);
	inspector_vb->set_anchors_preset(PRESET_WIDE);
	add_child(inspector_vb);

	Label *label = memnew(Label);
	inspector_vb->add_child(label);
	label->set_h_size_flags(SIZE_EXPAND_FILL);
	label->set_text(TTR("Selected particles:"));

	inspector_particles_txt = memnew(RichTextLabel);
	inspector_vb->add_child(inspector_particles_txt);
	inspector_particles_txt->set_h_size_flags(SIZE_EXPAND_FILL);
	inspector_particles_txt->set_custom_minimum_size(Size2(0, 200));
	inspector_particles_txt->set_text("");

	make_spin_box(inspector_mass_inp, 0, 100, 0.01, 1, inspector_vb, TTR("Particle mass"));
	inspector_mass_inp->connect("value_changed", this, "_mass_changed");

	redraw();
}

void ParticleBodyEditor::edit(ParticleBody *p_body) {

	node = p_body;
	if (!node)
		return;

	show_gizmo_btn->set_pressed(node->draw_gizmo);
}

void ParticleBodyEditor::redraw() {

	if (!node)
		return;

	Ref<ParticleBodySpatialGizmo> gizmo = node->get_gizmo();

	if (gizmo.is_null())
		return;

	if (node->get_particle_body_model().is_null())
		return;

	if (!node->get_particle_body_model()->get_particles().size())
		return;

	PoolVector<real_t>::Read masses_read = node->get_particle_body_model()->get_masses().read();

	String particles = "";
	real_t masses = -1;

	for (int i = 0; i < gizmo->get_selected_particles().size(); ++i) {

		int particle = gizmo->get_selected_particles()[i];

		if (String() != particles)
			particles += ", ";

		particles += String::num(particle);

		if (masses == -1) {
			masses = masses_read[particle];
		} else if (masses != masses_read[particle]) {
			masses += masses_read[particle];
			masses /= 2;
		}
	}

	inspector_particles_txt->set_text(particles);

	inspector_mass_inp->disconnect("value_changed", this, "_mass_changed");
	inspector_mass_inp->set_value(masses);
	inspector_mass_inp->connect("value_changed", this, "_mass_changed");
}

void PhysicsParticleBodyEditorPlugin::edit(Object *p_object) {

	particle_body_editor->edit(Object::cast_to<ParticleBody>(p_object));
}

bool PhysicsParticleBodyEditorPlugin::handles(Object *p_object) const {

	return p_object->is_class("ParticleBody");
}

void PhysicsParticleBodyEditorPlugin::make_visible(bool p_visible) {

	if (p_visible) {

		particle_body_editor->options->show();
		particle_body_editor->show_gizmo_btn->show();
		if (particle_body_editor->show_gizmo_btn->is_pressed())
			particle_body_editor->show();
	} else {

		particle_body_editor->options->hide();
		particle_body_editor->show_gizmo_btn->hide();
		particle_body_editor->hide();
		particle_body_editor->edit(NULL);
	}
}

void PhysicsParticleBodyEditorPlugin::redraw() {
	particle_body_editor->redraw();
}

PhysicsParticleBodyEditorPlugin::PhysicsParticleBodyEditorPlugin(EditorNode *p_node) :
		EditorPlugin(),
		editor(p_node),
		particle_body_editor(memnew(ParticleBodyEditor)) {

	SpatialEditor::get_singleton()->get_palette_split()->add_child(particle_body_editor);
	SpatialEditor::get_singleton()->get_palette_split()->move_child(particle_body_editor, 1);

	make_visible(false);
}

PhysicsParticleBodyEditorPlugin::~PhysicsParticleBodyEditorPlugin() {
}
