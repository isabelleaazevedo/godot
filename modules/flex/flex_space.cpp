/*************************************************************************/
/*  flex_space.cpp                                                       */
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

#include "flex_space.h"

#include "thirdparty/flex/include/NvFlex.h"

#include "flex_memory.h"
#include "flex_particle_body.h"
#include "print_string.h"

#define MAXPARTICLES 1000

#define DEVICE_ID 0

// TODO use a class
NvFlexErrorSeverity error_severity = eNvFlexLogInfo; // contain last error severity
void ErrorCallback(NvFlexErrorSeverity severity, const char *msg, const char *file, int line) {
    print_error(String("Flex error file: ") + file + " - LINE: " + String::num(line) + " - MSG: " + msg);
    error_severity = severity;
}
bool has_error() {
    return error_severity == eNvFlexLogError;
}

FlexSpace::FlexSpace() :
        RIDFlex(),
        flex_lib(NULL),
        solver(NULL),
        particle_bodies_allocator(NULL),
        particle_bodies_memory(NULL),
        springs_allocator(NULL),
        springs_memory(NULL) {
    init();
}

FlexSpace::~FlexSpace() {
    terminate();
}

void FlexSpace::init() {

    // Init library
    ERR_FAIL_COND(flex_lib);

    NvFlexInitDesc desc;
    desc.deviceIndex = DEVICE_ID;
    desc.enableExtensions = true;
    desc.renderDevice = NULL;
    desc.renderContext = NULL;
    desc.computeContext = NULL;
    desc.computeType = eNvFlexCUDA;
    desc.runOnRenderContext = false;

    flex_lib = NvFlexInit(NV_FLEX_VERSION, ErrorCallback, &desc);
    ERR_FAIL_COND(!flex_lib);
    ERR_FAIL_COND(has_error());

    // Init solvert
    ERR_FAIL_COND(solver);

    NvFlexSolverDesc solver_desc;

    NvFlexSetSolverDescDefaults(&solver_desc);
    solver_desc.featureMode = eNvFlexFeatureModeDefault; // All modes enabled (Solid|Fluids) // TODO should be customizable
    solver_desc.maxParticles = MAXPARTICLES; // TODO should be customizable
    solver_desc.maxDiffuseParticles = MAXPARTICLES; // TODO should be customizable
    solver_desc.maxNeighborsPerParticle = 32; // TODO should be customizable
    solver_desc.maxContactsPerParticle = 10; // TODO should be customizable

    solver = NvFlexCreateSolver(flex_lib, &solver_desc);
    ERR_FAIL_COND(has_error());

    // Init buffers
    ERR_FAIL_COND(particle_bodies_memory);
    ERR_FAIL_COND(particle_bodies_allocator);
    particle_bodies_memory = memnew(ParticleBodiesMemory(flex_lib));
    particle_bodies_allocator = memnew(FlexMemoryAllocator(particle_bodies_memory, ((FlexUnit)(MAXPARTICLES / 3))));
    particle_bodies_memory->unmap(); // This is mandatory because the FlexMemoryAllocator when resize the memory will leave the buffers mapped

    ERR_FAIL_COND(springs_allocator);
    ERR_FAIL_COND(springs_memory);
    springs_memory = memnew(SpringMemory(flex_lib));
    springs_allocator = memnew(FlexMemoryAllocator(springs_memory, ((FlexUnit)(MAXPARTICLES * 2))));
    springs_memory->unmap(); // This is mandatory because the FlexMemoryAllocator when resize the memory will leave the buffers mapped

    NvFlexParams params;
    // Initialize solver parameter
    NvFlexGetParams(solver, &params);
    params.gravity[0] = 0.0;
    params.gravity[1] = -10.0;
    params.gravity[2] = 0.0;
    params.radius = 0.1;
    params.solidRestDistance = params.radius * 0.9;
    params.fluidRestDistance = params.radius * 0.5;
    params.numIterations = 3;
    params.maxSpeed = FLT_MAX;
    params.maxAcceleration = Vector3(params.gravity[0], params.gravity[1], params.gravity[2]).length() * 10.0;
    params.relaxationMode = eNvFlexRelaxationLocal;
    params.relaxationFactor = 1.0;
    params.solidPressure = 1.0;
    params.collisionDistance = MAX(params.solidRestDistance, params.fluidRestDistance) * 0.5;
    params.shapeCollisionMargin = params.collisionDistance * 0.5;
    params.particleFriction = params.dynamicFriction * 0.1;
    NvFlexSetParams(solver, &params);

    ERR_FAIL_COND(has_error());
}

void FlexSpace::terminate() {

    ERR_FAIL_COND(!particle_bodies_memory);
    particle_bodies_memory->terminate();
    memdelete(particle_bodies_memory);
    particle_bodies_memory = NULL;

    ERR_FAIL_COND(!particle_bodies_allocator);
    memdelete(particle_bodies_allocator);
    particle_bodies_allocator = NULL;

    ERR_FAIL_COND(!solver);
    NvFlexDestroySolver(solver);
    solver = NULL;

    ERR_FAIL_COND(!flex_lib);
    NvFlexShutdown(flex_lib);
    flex_lib = NULL;
}

void FlexSpace::sync() {

    particle_bodies_memory->map();
    springs_memory->map();

    dispatch_callbacks();
    execute_delayed_commands();

    particle_bodies_memory->unmap();

    if (springs_memory->was_changed())
        springs_allocator->sanitize(); // This buffer should be compact when the GPU has to use it
    springs_memory->unmap();

    commands_write_buffer();
}

void FlexSpace::step(real_t p_delta_time) {

    // Step solver (command)
    const int substep = 1;
    const bool enable_timer = false; // Used for profiling
    NvFlexUpdateSolver(solver, p_delta_time, substep, enable_timer);

    commands_read_buffer();
}

void FlexSpace::add_particle_body(FlexParticleBody *p_body) {
    ERR_FAIL_COND(p_body->space);
    p_body->space = this;
    particle_bodies.push_back(p_body);
}

void FlexSpace::remove_particle_body(FlexParticleBody *p_body) {
    // TODO implement
    print_error("remove_particle_body NOT YET IMPLEMENTED! ");
    //p_body->space = NULL;
    //particle_bodies.erase(p_body);
}

void FlexSpace::dispatch_callbacks() {
    for (int i(particle_bodies.size() - 1); 0 <= i; --i) {
        particle_bodies[i]->dispatch_sync_callback();
    }
}

void FlexSpace::execute_delayed_commands() {
    for (int i(particle_bodies.size() - 1); 0 <= i; --i) {

        FlexParticleBody *body = particle_bodies[i];
        if (body->delayed_commands.particle_to_add.size()) {

            int previous_size = 0;

            // Allocate memory for particles
            if (body->particles_mchunk) {
                previous_size = body->particles_mchunk->get_size();
                // Resize existing memory chunk
                particle_bodies_allocator->resize_chunk(body->particles_mchunk, previous_size + body->delayed_commands.particle_to_add.size());
            } else {
                // Allocate new one
                body->particles_mchunk = particle_bodies_allocator->allocate_chunk(body->delayed_commands.particle_to_add.size());
            }

            // Write on memory
            for (int p(body->delayed_commands.particle_to_add.size() - 1); 0 <= p; --p) {

                particle_bodies_memory->set_particle(body->particles_mchunk, previous_size + p, body->delayed_commands.particle_to_add[p].particle);
                particle_bodies_memory->set_velocity(body->particles_mchunk, previous_size + p, Vector3());
                particle_bodies_memory->set_phase(body->particles_mchunk, previous_size + p, NvFlexMakePhase(body->get_collision_group(), 0));
                particle_bodies_memory->set_active_particle(body->particles_mchunk, previous_size + p);
            }
        }

        if (body->delayed_commands.springs_to_add.size()) {

            int previous_size = 0;

            // Allocate memory for springs
            if (body->springs_mchunk) {
                previous_size = body->springs_mchunk->get_size();
                // Resize existing memory chunk
                springs_allocator->resize_chunk(body->springs_mchunk, previous_size + body->delayed_commands.springs_to_add.size());
            } else {
                // Allocate new chunk
                body->springs_mchunk = springs_allocator->allocate_chunk(body->delayed_commands.springs_to_add.size());
            }

            // Write on memory
            for (int s(body->delayed_commands.springs_to_add.size() - 1); 0 <= s; --s) {
                const SpringToAdd &sta(body->delayed_commands.springs_to_add[s]);
                body->reset_spring(s, sta.particle_0, sta.particle_1, sta.length, sta.stiffness);
            }
        }

        if (body->delayed_commands.particle_to_remove.size() && body->particles_mchunk) {

            // Remove particles AND find associated springs

            ParticleIndex last_buffer_index(body->particles_mchunk->get_end_index());
            for (Set<ParticleIndex>::Element *e = body->delayed_commands.particle_to_remove.front(); e; e = e->next()) {

                // Copy the values of last index to the index to remove (lose order)
                const ParticleBufferIndex buffer_index_to_remove(body->particles_mchunk->get_buffer_index(e->get()));

                // Find and remove springs associated to the particle to remove
                if (body->springs_mchunk) {
                    // Find all springs associated to removed particle and put in the remove list
                    for (int spring_index(body->springs_mchunk->get_size() - 1); 0 <= spring_index; --spring_index) {
                        const Spring &spring = springs_memory->get_spring(body->springs_mchunk, spring_index);
                        if (spring.index0 == buffer_index_to_remove || spring.index1 == buffer_index_to_remove) {
                            body->remove_spring(spring_index);
                        }
                    }
                }

                particle_bodies_memory->copy(last_buffer_index, 1, buffer_index_to_remove);
                replace_particle_index_in_springs(body, last_buffer_index, buffer_index_to_remove);
                body->particle_index_changed(body->particles_mchunk->get_chunk_index(last_buffer_index), e->get());
                --last_buffer_index;
            }
            const FlexUnit new_size = body->particles_mchunk->get_size() - body->delayed_commands.particle_to_remove.size();
            particle_bodies_allocator->resize_chunk(body->particles_mchunk, new_size);
        }

        if (body->delayed_commands.springs_to_remove.size() && body->springs_mchunk) {

            // Remove springs

            SpringIndex last_buffer_index(body->springs_mchunk->get_end_index());
            for (Set<SpringIndex>::Element *e = body->delayed_commands.springs_to_remove.front(); e; e = e->next()) {

                // Copy the values of last ID to the ID to remove (lose order)
                const SpringBufferIndex buffer_index_to_remove(body->springs_mchunk->get_buffer_index(e->get()));
                springs_memory->copy(last_buffer_index, 1, buffer_index_to_remove);
                body->spring_index_changed(body->springs_mchunk->get_chunk_index(last_buffer_index), e->get());
                --last_buffer_index;
            }
            const FlexUnit new_size = body->springs_mchunk->get_size() - body->delayed_commands.springs_to_remove.size();
            springs_allocator->resize_chunk(body->springs_mchunk, new_size);
        }

        // Apply changed properties
        const uint32_t body_changed_parameters = body->get_changed_parameters();
        if (body_changed_parameters != 0) {
            for (int i(body->get_particle_count() - 1); 0 <= i; --i) {
                if (body_changed_parameters & eChangedParameterGroup) {
                    particle_bodies_memory->set_phase(body->particles_mchunk, i, NvFlexMakePhase(body->group, 0));
                }
            }
        }
    }
}

void FlexSpace::commands_write_buffer() {
    NvFlexCopyDesc copy_desc;
    for (int i(particle_bodies.size() - 1); 0 <= i; --i) {

        FlexParticleBody *body = particle_bodies[i];
        if (!body->particles_mchunk)
            continue;

        const uint32_t changed_params(body->get_changed_parameters());
        if (changed_params != 0) {
            copy_desc.srcOffset = body->particles_mchunk->get_begin_index();
            copy_desc.dstOffset = body->particles_mchunk->get_begin_index();
            copy_desc.elementCount = body->particles_mchunk->get_size();

            if (changed_params & eChangedParameterPositionMass)
                NvFlexSetParticles(solver, particle_bodies_memory->particles.buffer, &copy_desc);
            if (changed_params & eChangedParameterVelocity)
                NvFlexSetVelocities(solver, particle_bodies_memory->velocities.buffer, &copy_desc);
            if (changed_params & eChangedParameterGroup)
                NvFlexSetPhases(solver, particle_bodies_memory->phases.buffer, &copy_desc);
            if (changed_params & eChangedParameterActive) {
                NvFlexSetActive(solver, particle_bodies_memory->active_particles.buffer, &copy_desc);
                NvFlexSetActiveCount(solver, particle_bodies_memory->active_particles.size());
            }

            body->reset_changed_parameters();
        }
    }

    if (springs_memory->was_changed())
        NvFlexSetSprings(solver, springs_memory->springs.buffer, springs_memory->lengths.buffer, springs_memory->stiffness.buffer, springs_allocator->get_last_used_index() + 1);
}

void FlexSpace::commands_read_buffer() {
    NvFlexCopyDesc copy_desc;
    for (int i(particle_bodies.size() - 1); 0 <= i; --i) {

        if (!particle_bodies[i]->particles_mchunk)
            continue;

        // Write back to buffer (command)
        copy_desc.srcOffset = particle_bodies[i]->particles_mchunk->get_begin_index();
        copy_desc.dstOffset = particle_bodies[i]->particles_mchunk->get_begin_index();
        copy_desc.elementCount = particle_bodies[i]->particles_mchunk->get_size();

        // TODO read only necessary (part of buffer or just skip an entire buffer if not necessary)
        NvFlexGetParticles(solver, particle_bodies_memory->particles.buffer, &copy_desc);
        NvFlexGetVelocities(solver, particle_bodies_memory->velocities.buffer, &copy_desc);
        NvFlexGetPhases(solver, particle_bodies_memory->phases.buffer, &copy_desc);

        particle_bodies[i]->clear_commands();
    }
}

void FlexSpace::replace_particle_index_in_springs(FlexParticleBody *p_body, ParticleBufferIndex p_index_old, ParticleBufferIndex p_index_new) {
    // Update the springs order
    for (int i(p_body->springs_mchunk->get_size() - 1); 0 <= i; --i) {

        const Spring &spring(springs_memory->get_spring(p_body->springs_mchunk, i));
        if (spring.index0 == p_index_old) {

            springs_memory->set_spring(p_body->springs_mchunk, i, Spring(p_index_new, spring.index1));
        } else if (spring.index1 == p_index_old) {

            springs_memory->set_spring(p_body->springs_mchunk, i, Spring(spring.index0, p_index_new));
        }

        const Spring &spring1(springs_memory->get_spring(p_body->springs_mchunk, i));
        const Spring &spring2(springs_memory->get_spring(p_body->springs_mchunk, i));
    }
}
