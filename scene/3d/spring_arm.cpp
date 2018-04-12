/*************************************************************************/
/*  spring_arm.h                                                         */
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

#include "spring_arm.h"
#include "servers/physics_server.h"
#include "scene/resources/sphere_shape.h"
#include "scene/3d/collision_object.h"


SpringArm::SpringArm() {
	spring_max_length = 10;
	spring_max_height = 5;
	smoothness = 0.5;
	looking_at_target = true;
	target = NULL;
	exclude_target_children = false;
	shape = memnew(SphereShape());
	Object::cast_to<SphereShape>(shape)->set_radius(0.3);
	excluded_colliders = Set<RID>();
	exclude_target = true;
}

void SpringArm::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			if(target==NULL){
				target = NULL;
				set_target(NodePath());
				return;
			}
			set_target(target->get_path());
		} break;
		case NOTIFICATION_INTERNAL_PROCESS: {
			_process(get_process_delta_time());
		} break;
	}
}

void SpringArm::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_target", "target"), &SpringArm::set_target);
	ClassDB::bind_method(D_METHOD("set_spring_max_length", "length"), &SpringArm::set_spring_max_length);
	ClassDB::bind_method(D_METHOD("set_spring_max_height", "height"), &SpringArm::set_spring_max_height);
	ClassDB::bind_method(D_METHOD("set_smoothness", "smoothness"), &SpringArm::set_smoothness);
	ClassDB::bind_method(D_METHOD("set_looking_at_target", "look_at_target"), &SpringArm::set_looking_at_target);

	ClassDB::bind_method(D_METHOD("get_target"), &SpringArm::get_target);
	ClassDB::bind_method(D_METHOD("get_spring_max_length"), &SpringArm::get_spring_max_length);
	ClassDB::bind_method(D_METHOD("get_spring_max_height"), &SpringArm::get_spring_max_height);
	ClassDB::bind_method(D_METHOD("get_smoothness"), &SpringArm::get_smoothness);
	ClassDB::bind_method(D_METHOD("is_looking_at_target"), &SpringArm::is_looking_at_target);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "target"), "set_target", "get_target");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "spring_max_length"), "set_spring_max_length", "get_spring_max_length");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "spring_max_height"), "set_spring_max_height", "get_spring_max_height");
	ADD_PROPERTY(PropertyInfo(Variant::REAL, "smoothness"), "set_smoothness", "get_smoothness");
	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "looking_at_target"), "set_looking_at_target", "is_looking_at_target");
}

float SpringArm::get_spring_max_length() const {
	return spring_max_length;
}

void SpringArm::set_spring_max_length(float p_length) {
	spring_max_length = p_length;
}

float SpringArm::get_spring_max_height() const {
	return spring_max_height;
}

void SpringArm::set_spring_max_height(float p_height) {
	spring_max_height = p_height;
}

float SpringArm::get_smoothness() const {
	return smoothness;
}

void SpringArm::set_smoothness(float p_smoothness) {
	this->smoothness = CLAMP(smoothness, 0.001, 1.0);
}


void SpringArm::set_looking_at_target(bool p_look_at_target) {
	looking_at_target = p_look_at_target;
}

bool SpringArm::is_looking_at_target() const {
	return looking_at_target;
}

NodePath SpringArm::get_target() const {
	if (!target)
		return NodePath();

	return get_path_to(target);
}

void SpringArm::set_target(const NodePath &p_target) {

	Spatial *target_ptr = Object::cast_to<Spatial>(get_node(p_target));


	if (!target_ptr || target == this) {
		set_process_internal(false);
		this->target = NULL;
		return;
	}
	set_process_internal(true);
	this->target = target_ptr;
	CollisionObject *collision = Object::cast_to<CollisionObject>(target_ptr);
	if(!collision){
		return;
	}
	excluded_colliders = Set<RID>();
	if(!exclude_target_children){
		return;
	}
	//TODO children
	Vector<Object> to_explore = Vector<Object>();
	
}

void SpringArm::_process(float p_delta) {

    Vector3 wanted_position = _compute_movement();
	//print_line("position is "+ get_global_transform().origin.operator String() + 
	//	" wanted position "+ wanted_position.operator String());
	Vector3 motion_vec = wanted_position - get_global_transform().origin;
	PhysicsDirectSpaceState::ShapeRestInfo info = {};
	float closest_safe=1.0, closest_unsafe;

    bool motion_possible = get_world()->get_direct_space_state()->cast_motion(shape->get_rid(),
	 	get_global_transform(), 
		motion_vec, 0.1 , closest_safe, closest_unsafe, 
		excluded_colliders, 0x7FFFFFFF,
		&info);

	if(!motion_possible){
		print_line("No motion possible");
		return;
	}
	print_line("motion possible");
	motion_vec *= closest_safe;
	motion_vec.slide(info.normal);
	Transform trans = get_global_transform();
	trans.origin += motion_vec;
	set_global_transform(trans);
	
}

Vector3 SpringArm::_compute_movement() {
	Vector3 target_forward = -target->get_global_transform().basis.get_axis(2).normalized();
	Vector3 target_up = target->get_global_transform().basis.get_axis(1).normalized();
	Vector3 camera_position = target->get_global_transform().origin;
	camera_position += -target_forward * spring_max_length + target_up * spring_max_height;
	return camera_position;
}