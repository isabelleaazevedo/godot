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

#ifndef SPRING_ARM_H
#define SPRING_ARM_H

#include "scene/3d/spatial.h"


class SpringArm : public Spatial {
	GDCLASS(SpringArm, Spatial);

    
	Spatial *target;
    Shape *shape;

	float spring_max_length;
	float spring_max_height;
	float smoothness;
	bool looking_at_target;
	bool exclude_target_children;
	bool exclude_target;
	Set<RID> excluded_colliders;

protected:
	void _notification(int p_what);
	static void _bind_methods();

public:
	void set_target(const NodePath &p_target);
	NodePath get_target() const;
	void set_spring_max_length(float p_length);
	float get_spring_max_length() const;
	void set_spring_max_height(float p_height);
	float get_spring_max_height() const;
	void set_smoothness(float p_smoothness);
	float get_smoothness() const;
	void set_looking_at_target(bool p_look_at_target);
	bool is_looking_at_target() const;
	void set_shape(const Shape *p_shape);
	Shape* get_shape() const;

	SpringArm();

private:
	void _process(float p_delta);
	Vector3 _compute_movement();
};

#endif