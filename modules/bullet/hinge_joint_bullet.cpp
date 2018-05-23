/*************************************************************************/
/*  hinge_joint_bullet.cpp                                               */
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

#include "hinge_joint_bullet.h"

#include "bullet_types_converter.h"
#include "bullet_utilities.h"
#include "rigid_body_bullet.h"

#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>

/**
	@author AndreaCatania
*/

HingeJointBullet::HingeJointBullet(RigidBodyBullet *rbA, RigidBodyBullet *rbB, const Transform &frameA, const Transform &frameB) :
		JointBullet() {

	construct(rbA, rbB, frameA, frameB);
}

void make_frame(const Vector3 &p_pivotInA, const Vector3 &p_axisInA, Transform &r_frame) {
	Vector3 Z = p_axisInA;
	Vector3 X = Vector3(0, 1, 0).cross(Z).normalized();
	Vector3 Y = X.cross(p_axisInA).normalized();

	r_frame.basis.set(X, Y, Z);
	r_frame.set_origin(p_pivotInA);
}

HingeJointBullet::HingeJointBullet(RigidBodyBullet *rbA, RigidBodyBullet *rbB, const Vector3 &pivotInA, const Vector3 &pivotInB, const Vector3 &axisInA, const Vector3 &axisInB) :
		JointBullet() {

	Transform frameA;
	Transform frameB;
	make_frame(pivotInA, axisInA, frameA);
	make_frame(pivotInB, axisInB, frameB);

	construct(rbA, rbB, frameA, frameB);
}

void HingeJointBullet::construct(RigidBodyBullet *rbA, RigidBodyBullet *rbB, const Transform &frameA, const Transform &frameB) {
	Transform scaled_AFrame(frameA.scaled(rbA->get_body_scale()));
	scaled_AFrame.basis.rotref_posscale_decomposition(scaled_AFrame.basis);

	btTransform btFrameA;
	G_TO_B(scaled_AFrame, btFrameA);

	if (rbB) {

		Transform scaled_BFrame(frameB.scaled(rbB->get_body_scale()));
		scaled_BFrame.basis.rotref_posscale_decomposition(scaled_BFrame.basis);

		btTransform btFrameB;
		G_TO_B(scaled_BFrame, btFrameB);

		fakeHingeConstraint = bulletnew(btGeneric6DofConstraint(*rbA->get_bt_rigid_body(), *rbB->get_bt_rigid_body(), btFrameA, btFrameB, true));
	} else {

		fakeHingeConstraint = bulletnew(btGeneric6DofConstraint(*rbA->get_bt_rigid_body(), btFrameA, true));
	}

	fakeHingeConstraint->setLinearUpperLimit(btVector3(0, 0, 0));
	fakeHingeConstraint->setLinearLowerLimit(btVector3(0, 0, 0));

	fakeHingeConstraint->setAngularUpperLimit(btVector3(0, 0, Math_PI));
	fakeHingeConstraint->setAngularLowerLimit(btVector3(0, 0, -Math_PI));

	setup(fakeHingeConstraint);
}

real_t HingeJointBullet::get_hinge_angle() {
	return 0; //fakeHingeConstraint->getHingeAngle();
}

void HingeJointBullet::set_param(PhysicsServer::HingeJointParam p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer::HINGE_JOINT_BIAS:
			if (0 < p_value) {
				print_line("The Bullet Hinge Joint doesn't support bias, So it's always 0");
			}
			break;
		case PhysicsServer::HINGE_JOINT_LIMIT_UPPER:
			fakeHingeConstraint->setAngularUpperLimit(btVector3(0, 0, p_value));
			break;
		case PhysicsServer::HINGE_JOINT_LIMIT_LOWER:
			fakeHingeConstraint->setAngularLowerLimit(btVector3(0, 0, p_value));
			break;
		case PhysicsServer::HINGE_JOINT_LIMIT_BIAS:
			break;
		case PhysicsServer::HINGE_JOINT_LIMIT_SOFTNESS:
			break;
		case PhysicsServer::HINGE_JOINT_LIMIT_RELAXATION:
			break;
		case PhysicsServer::HINGE_JOINT_MOTOR_TARGET_VELOCITY:
			fakeHingeConstraint->getRotationalLimitMotor(2)->m_targetVelocity = p_value;
			break;
		case PhysicsServer::HINGE_JOINT_MOTOR_MAX_IMPULSE:
			fakeHingeConstraint->getRotationalLimitMotor(2)->m_maxMotorForce = p_value;
			break;
		default:
			WARN_PRINTS("The Bullet Hinge Joint doesn't support this parameter: " + itos(p_param) + ", value: " + itos(p_value));
	}
}

real_t HingeJointBullet::get_param(PhysicsServer::HingeJointParam p_param) const {
	switch (p_param) {
		case PhysicsServer::HINGE_JOINT_BIAS:
			return 0;
			break;
		case PhysicsServer::HINGE_JOINT_LIMIT_UPPER: {
			btVector3 lim;
			fakeHingeConstraint->getAngularUpperLimit(lim);
			return lim.getZ();
		}
		case PhysicsServer::HINGE_JOINT_LIMIT_LOWER: {
			btVector3 lim;
			fakeHingeConstraint->getAngularLowerLimit(lim);
			return lim.getZ();
		}
		case PhysicsServer::HINGE_JOINT_LIMIT_BIAS:
			return 0;
		case PhysicsServer::HINGE_JOINT_LIMIT_SOFTNESS:
			return 0;
		case PhysicsServer::HINGE_JOINT_LIMIT_RELAXATION:
			return 0;
		case PhysicsServer::HINGE_JOINT_MOTOR_TARGET_VELOCITY:
			return fakeHingeConstraint->getRotationalLimitMotor(2)->m_targetVelocity;
		case PhysicsServer::HINGE_JOINT_MOTOR_MAX_IMPULSE:
			return fakeHingeConstraint->getRotationalLimitMotor(2)->m_maxMotorForce;
		default:
			WARN_PRINTS("The Bullet Hinge Joint doesn't support this parameter: " + itos(p_param));
			return 0;
	}
}

void HingeJointBullet::set_flag(PhysicsServer::HingeJointFlag p_flag, bool p_value) {
	switch (p_flag) {
		case PhysicsServer::HINGE_JOINT_FLAG_USE_LIMIT:
			if (!p_value) {
				fakeHingeConstraint->setAngularUpperLimit(btVector3(0, 0, Math_PI));
				fakeHingeConstraint->setAngularLowerLimit(btVector3(0, 0, -Math_PI));
			}
			break;
		case PhysicsServer::HINGE_JOINT_FLAG_ENABLE_MOTOR:
			fakeHingeConstraint->getRotationalLimitMotor(2)->m_enableMotor = p_value;
			break;
	}
}

bool HingeJointBullet::get_flag(PhysicsServer::HingeJointFlag p_flag) const {
	switch (p_flag) {
		case PhysicsServer::HINGE_JOINT_FLAG_USE_LIMIT:
			return get_param(PhysicsServer::HINGE_JOINT_LIMIT_UPPER) != 0 && get_param(PhysicsServer::HINGE_JOINT_LIMIT_LOWER) != 0;
		case PhysicsServer::HINGE_JOINT_FLAG_ENABLE_MOTOR:
			return fakeHingeConstraint->getRotationalLimitMotor(2)->m_enableMotor;
		default:
			return false;
	}
}
