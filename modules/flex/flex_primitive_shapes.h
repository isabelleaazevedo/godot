/*************************************************************************/
/*  flex_primitive_shapes.cpp                                            */
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
	@author AndreaCatania
*/

#ifndef FLEX_PRIMITIVE_SHAPES_H
#define FLEX_PRIMITIVE_SHAPES_H

#include "core/variant.h"
#include "flex_utility.h"
#include "rid_flex.h"
#include "thirdparty/flex/include/NvFlexExt.h"

class FlexPrimitiveShape : public RIDFlex {

public:
	FlexPrimitiveShape();

	virtual NvFlexCollisionShapeType get_type() = 0;
	virtual void get_shape(NvFlexCollisionGeometry *r_shape) const = 0;
	virtual void set_data(const Variant &p_data) = 0;
	virtual Variant get_data() const = 0;
};

class FlexPrimitiveBoxShape : public FlexPrimitiveShape {

	Vector3 extends;

public:
	FlexPrimitiveBoxShape();

	virtual NvFlexCollisionShapeType get_type() { return eNvFlexShapeBox; }
	virtual void get_shape(NvFlexCollisionGeometry *r_shape) const;
	virtual void set_data(const Variant &p_data);
	virtual Variant get_data() const;

	void set_extends(const Vector3 &p_extends);
	const Vector3 &get_extends() const { return extends; }
};

#endif // FLEX_PRIMITIVE_SHAPES_H
