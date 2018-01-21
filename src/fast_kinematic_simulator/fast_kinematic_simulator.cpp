#include <fast_kinematic_simulator/simple_particle_contact_simulator.hpp>
#include <fast_kinematic_simulator/fast_kinematic_simulator.hpp>

uncertainty_planning_core::SE2SimulatorPtr
fast_kinematic_simulator::MakeSE2Simulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment,
                                           const sdf_tools::SignedDistanceField& environment_sdf,
                                           const simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid,
                                           const SolverParameters& solver_config,
                                           const double simulation_controller_frequency,
                                           const uint64_t prng_seed,
                                           const int32_t debug_level)
{
    return uncertainty_planning_core::SE2SimulatorPtr(
                new simple_particle_contact_simulator::SimpleParticleContactSimulator<tnuva_robot_models::TnuvaSE2Robot<uncertainty_planning_core::PRNG>,
                                                                                      uncertainty_planning_core::SE2Config,
                                                                                      uncertainty_planning_core::PRNG,
                                                                                      uncertainty_planning_core::SE2ConfigAlloc>(environment,
                                                                                                                                 environment_sdf,
                                                                                                                                 surface_normals_grid,
                                                                                                                                 solver_config,
                                                                                                                                 simulation_controller_frequency,
                                                                                                                                 false,
                                                                                                                                 prng_seed,
                                                                                                                                 debug_level));
}

uncertainty_planning_core::SE3SimulatorPtr
fast_kinematic_simulator::MakeSE3Simulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment,
                                           const sdf_tools::SignedDistanceField& environment_sdf,
                                           const simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid,
                                           const SolverParameters& solver_config,
                                           const double simulation_controller_frequency,
                                           const uint64_t prng_seed,
                                           const int32_t debug_level)
{
    return uncertainty_planning_core::SE3SimulatorPtr(
                new simple_particle_contact_simulator::SimpleParticleContactSimulator<tnuva_robot_models::TnuvaSE3Robot<uncertainty_planning_core::PRNG>,
                                                                                      uncertainty_planning_core::SE3Config,
                                                                                      uncertainty_planning_core::PRNG,
                                                                                      uncertainty_planning_core::SE3ConfigAlloc>(environment,
                                                                                                                                 environment_sdf,
                                                                                                                                 surface_normals_grid,
                                                                                                                                 solver_config,
                                                                                                                                 simulation_controller_frequency,
                                                                                                                                 false,
                                                                                                                                 prng_seed,
                                                                                                                                 debug_level));
}

uncertainty_planning_core::LinkedSimulatorPtr
fast_kinematic_simulator::MakeLinkedSimulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment,
                                              const sdf_tools::SignedDistanceField& environment_sdf,
                                              const simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid,
                                              const SolverParameters& solver_config,
                                              const double simulation_controller_frequency,
                                              const uint64_t prng_seed,
                                              const int32_t debug_level)
{
    return uncertainty_planning_core::LinkedSimulatorPtr(
                new simple_particle_contact_simulator::SimpleParticleContactSimulator<tnuva_robot_models::TnuvaLinkedRobot<uncertainty_planning_core::PRNG>,
                                                                                      uncertainty_planning_core::LinkedConfig,
                                                                                      uncertainty_planning_core::PRNG,
                                                                                      uncertainty_planning_core::LinkedConfigAlloc>(environment,
                                                                                                                                    environment_sdf,
                                                                                                                                    surface_normals_grid,
                                                                                                                                    solver_config,
                                                                                                                                    simulation_controller_frequency,
                                                                                                                                    false,
                                                                                                                                    prng_seed,
                                                                                                                                    debug_level));
}
