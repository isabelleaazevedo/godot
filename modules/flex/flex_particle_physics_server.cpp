/*************************************************************************/
/*  flex_particle_physics_server.cpp                                     */
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

#include "flex_particle_physics_server.h"

#include "thirdparty/flex/include/NvFlex.h"

#include "string.h"

/**
	@author AndreaCatania
*/

#define DEVICE 0
#define FS FlexParticlePhysicsServer::singleton

FlexBuffers::FlexBuffers() :
		// Allocation is managed automatically
		positions(FS->flex_lib),
		velocities(FS->flex_lib),
		phases(FS->flex_lib) {

	ERR_EXPLAIN("ERROR! Before initialize buffers you must initialize Flex library.");
	ERR_FAIL_COND(!FS->flex_lib);
}

void FlexBuffers::map() {
	positions.map(eNvFlexMapWait);
	velocities.map(eNvFlexMapWait);
	phases.map(eNvFlexMapWait);
}

void FlexBuffers::unmap() {
	positions.unmap();
	velocities.unmap();
	phases.unmap();
}

// TODO use a class
NvFlexErrorSeverity error_severity; // contain last error severity
void ErrorCallback(NvFlexErrorSeverity severity, const char *msg, const char *file, int line) {
	print_error(String("Flex error: ") + msg + ", FILE: " + file + ", LINE: " + String::num(line, 0));
	error_severity = severity;
}
bool has_error() {
	return error_severity == NvFlexErrorSeverity::eNvFlexLogError;
}

FlexParticlePhysicsServer *FlexParticlePhysicsServer::singleton = NULL;

void FlexParticlePhysicsServer::init() {

	// Init library
	ERR_FAIL_COND(flex_lib);

	NvFlexInitDesc desc;
	desc.deviceIndex = DEVICE;
	desc.enableExtensions = true;
	desc.renderDevice = DEVICE;
	desc.renderContext = 0;
	desc.computeContext = 0;
	desc.computeType = eNvFlexCUDA;
	desc.runOnRenderContext = false;

	flex_lib = NvFlexInit(NV_FLEX_VERSION, ErrorCallback, &desc);
	ERR_FAIL_COND(!flex_lib);
	ERR_FAIL_COND(!has_error());

	// Init solvert
	ERR_FAIL_COND(solver);

	NvFlexSolverDesc solver_desc;
	solver_desc.featureMode = eNvFlexFeatureModeDefault; // All modes enabled (Solid|Fluids) // TODO should be customizable
	solver_desc.maxParticles = 1000; // TODO should be customizable
	solver_desc.maxDiffuseParticles = 1000; // TODO should be customizable
	solver_desc.maxNeighborsPerParticle = 32; // TODO should be customizable
	solver_desc.maxContactsPerParticle = 10; // TODO should be customizable

	solver = NvFlexCreateSolver(flex_lib, &solver_desc);

	buffers = memnew(FlexBuffers);
}

void FlexParticlePhysicsServer::terminate() {

	ERR_FAIL_COND(!buffers);
	memdelete(buffers);
	buffers = NULL;

	ERR_FAIL_COND(!solver);
	NvFlexDestroySolver(solver);
	solver = NULL;

	ERR_FAIL_COND(!flex_lib);
	NvFlexShutdown(flex_lib);
	flex_lib = NULL;
}

void FlexParticlePhysicsServer::sync() {

	// Wait for previous iteration to finish then get access to solver buffer
	//float4 *particles = (float4 *)NvFlexMap(particleBuffer, eNvFlexMapWait);
	//float3 *velocities = (float3 *)NvFlexMap(velocityBuffer, eNvFlexMapWait);
	//int *phases = (int *)NvFlexMap(phaseBuffer, eNvFlexMapWait);
}

void FlexParticlePhysicsServer::flush_queries() {

	/// PERFORM WRITE OPERATIONS
	// Add / Remove particle body from buffers

	// Chane particle
	//UpdateBuffers(particles, velocities, phases);

	// Get here particle body info (Like position etc..)

	// unmap buffers
	//NvFlexUnmap(particleBuffer);
	//NvFlexUnmap(velocityBuffer);
	//NvFlexUnmap(phaseBuffer);
}

void FlexParticlePhysicsServer::step(real_t p_delta_time) {
	//print_line(String::num(p_delta_time) + " <- Particle P server DT");
	// Step solver
}

FlexParticlePhysicsServer::FlexParticlePhysicsServer() :
		ParticlePhysicsServer(),
		flex_lib(NULL) {
	ERR_FAIL_COND(singleton);
	singleton = this;
}

FlexParticlePhysicsServer::~FlexParticlePhysicsServer() {
}
