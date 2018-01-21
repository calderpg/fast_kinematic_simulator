#include <uncertainty_planning_core/simple_simulator_interface.hpp>
#include <fast_kinematic_simulator/simple_particle_contact_simulator.hpp>
#include <fast_kinematic_simulator/tnuva_robot_models.hpp>
#include <uncertainty_planning_core/uncertainty_planning_core.hpp>

#ifndef FAST_KINEMATIC_SIMULATOR_HPP
#define FAST_KINEMATIC_SIMULATOR_HPP

namespace fast_kinematic_simulator
{
    typedef simple_particle_contact_simulator::SimulatorSolverParameters SolverParameters;

    inline SolverParameters GetDefaultSolverParameters()
    {
        return SolverParameters();
    }

    uncertainty_planning_core::SE2SimulatorPtr MakeSE2Simulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid, const SolverParameters& solver_config, const double simulation_controller_frequency, const uint64_t prng_seed, const int32_t debug_level);

    uncertainty_planning_core::SE3SimulatorPtr MakeSE3Simulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid, const SolverParameters& solver_config, const double simulation_controller_frequency, const uint64_t prng_seed, const int32_t debug_level);

    uncertainty_planning_core::LinkedSimulatorPtr MakeLinkedSimulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid, const SolverParameters& solver_config, const double simulation_controller_frequency, const uint64_t prng_seed, const int32_t debug_level);
}

#endif // FAST_KINEMATIC_SIMULATOR_HPP
