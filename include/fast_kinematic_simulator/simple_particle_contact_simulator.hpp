#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <vector>
#include <string>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <functional>
#include <chrono>
#include <random>
#include <atomic>
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <arc_utilities/simple_robot_models.hpp>
#include <uncertainty_planning_core/simple_simulator_interface.hpp>
#include <fast_kinematic_simulator/tnuva_robot_models.hpp>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <omp.h>

#ifndef SIMPLE_PARTICLE_CONTACT_SIMULATOR_HPP
#define SIMPLE_PARTICLE_CONTACT_SIMULATOR_HPP

namespace std
{
    template <>
    struct hash<std::pair<size_t, size_t>>
    {
        std::size_t operator()(const std::pair<size_t, size_t>& pair) const
        {
            using std::size_t;
            using std::hash;
            return (std::hash<size_t>()(pair.first) ^ (std::hash<size_t>()(pair.second) << 1));
        }
    };
}

namespace simple_particle_contact_simulator
{
    class SurfaceNormalGrid
    {
    protected:

        struct StoredSurfaceNormal
        {
        protected:

            Eigen::Vector4d entry_direction_;
            Eigen::Vector3d normal_;

        public:

            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

            StoredSurfaceNormal(const Eigen::Vector3d& normal, const Eigen::Vector3d& direction) : normal_(EigenHelpers::SafeNormal(normal))
            {
                const Eigen::Vector4d direction4d(direction.x(), direction.y(), direction.z(), 0.0);
                entry_direction_ = (EigenHelpers::SafeNormal(direction4d));
            }

            StoredSurfaceNormal(const Eigen::Vector3d& normal, const Eigen::Vector4d& direction) : entry_direction_(EigenHelpers::SafeNormal(direction)), normal_(EigenHelpers::SafeNormal(normal)) {}

            StoredSurfaceNormal() : entry_direction_(Eigen::Vector4d(0.0, 0.0, 0.0, 0.0)), normal_(Eigen::Vector3d(0.0, 0.0, 0.0)) {}

            const Eigen::Vector4d& EntryDirection4d() const
            {
                return entry_direction_;
            }

            Eigen::Vector3d EntryDirection3d() const
            {
                return entry_direction_.block<3, 1>(0, 0);
            }

            const Eigen::Vector3d& Normal() const
            {
                return normal_;
            }
        };

        VoxelGrid::VoxelGrid<std::vector<StoredSurfaceNormal>> surface_normal_grid_;
        bool initialized_;

        static Eigen::Vector3d GetBestSurfaceNormal(const std::vector<StoredSurfaceNormal>& stored_surface_normals, const Eigen::Vector3d& direction)
        {
            assert(stored_surface_normals.size() > 0);
            const double direction_norm = direction.norm();
            assert(direction_norm > 0.0);
            const Eigen::Vector3d unit_direction = direction / direction_norm;
            int32_t best_stored_index = -1;
            double best_dot_product = -std::numeric_limits<double>::infinity();
            for (size_t idx = 0; idx < stored_surface_normals.size(); idx++)
            {
                const StoredSurfaceNormal& stored_surface_normal = stored_surface_normals[idx];
                const double dot_product = stored_surface_normal.EntryDirection3d().dot(unit_direction);
                if (dot_product > best_dot_product)
                {
                    best_dot_product = dot_product;
                    best_stored_index = (int32_t)idx;
                }
            }
            assert(best_stored_index >= 0);
            const Eigen::Vector3d& best_surface_normal = stored_surface_normals[(size_t)best_stored_index].Normal();
            return best_surface_normal;
        }

        static Eigen::Vector3d GetBestSurfaceNormal(const std::vector<StoredSurfaceNormal>& stored_surface_normals, const Eigen::Vector4d& direction)
        {
            assert(stored_surface_normals.size() > 0);
            const double direction_norm = direction.norm();
            assert(direction_norm > 0.0);
            const Eigen::Vector4d unit_direction = direction / direction_norm;
            int32_t best_stored_index = -1;
            double best_dot_product = -std::numeric_limits<double>::infinity();
            for (size_t idx = 0; idx < stored_surface_normals.size(); idx++)
            {
                const StoredSurfaceNormal& stored_surface_normal = stored_surface_normals[idx];
                const double dot_product = stored_surface_normal.EntryDirection4d().dot(unit_direction);
                if (dot_product > best_dot_product)
                {
                    best_dot_product = dot_product;
                    best_stored_index = (int32_t)idx;
                }
            }
            assert(best_stored_index >= 0);
            const Eigen::Vector3d& best_surface_normal = stored_surface_normals[(size_t)best_stored_index].Normal();
            return best_surface_normal;
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SurfaceNormalGrid(const Eigen::Isometry3d& origin_transform, const double resolution, const double x_size, const double y_size, const double z_size)
        {
            surface_normal_grid_ = VoxelGrid::VoxelGrid<std::vector<StoredSurfaceNormal>>(origin_transform, resolution, x_size, y_size, z_size, std::vector<StoredSurfaceNormal>());
            initialized_ = true;
        }

        SurfaceNormalGrid() : initialized_(false) {}

        bool IsInitialized() const
        {
            return initialized_;
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const double x, const double y, const double z, const Eigen::Vector3d& direction) const
        {
            assert(initialized_);
            const Eigen::Vector4d location(x, y, z, 1.0);
            return LookupSurfaceNormal(location, direction);
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const Eigen::Vector3d& location, const Eigen::Vector3d& direction) const
        {
            assert(initialized_);
            const VoxelGrid::GRID_INDEX index = surface_normal_grid_.LocationToGridIndex3d(location);
            if (surface_normal_grid_.IndexInBounds(index))
            {
                return LookupSurfaceNormal(index, direction);
            }
            else
            {
                return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), false);
            }
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const Eigen::Vector4d& location, const Eigen::Vector3d& direction) const
        {
            assert(initialized_);
            const VoxelGrid::GRID_INDEX index = surface_normal_grid_.LocationToGridIndex4d(location);
            if (surface_normal_grid_.IndexInBounds(index))
            {
                return LookupSurfaceNormal(index, direction);
            }
            else
            {
                return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), false);
            }
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const Eigen::Vector4d& location, const Eigen::Vector4d& direction) const
        {
            assert(initialized_);
            const VoxelGrid::GRID_INDEX index = surface_normal_grid_.LocationToGridIndex4d(location);
            if (surface_normal_grid_.IndexInBounds(index))
            {
                return LookupSurfaceNormal(index, direction);
            }
            else
            {
                return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), false);
            }
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const VoxelGrid::GRID_INDEX& index, const Eigen::Vector3d& direction) const
        {
            assert(initialized_);
            return LookupSurfaceNormal(index.x, index.y, index.z, direction);
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const VoxelGrid::GRID_INDEX& index, const Eigen::Vector4d& direction) const
        {
            assert(initialized_);
            return LookupSurfaceNormal(index.x, index.y, index.z, direction);
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const int64_t x_index, const int64_t y_index, const int64_t z_index, const Eigen::Vector3d& direction) const
        {
            assert(initialized_);
            const std::pair<const std::vector<StoredSurfaceNormal>&, bool> lookup = surface_normal_grid_.GetImmutable(x_index, y_index, z_index);
            if (lookup.second)
            {
                const std::vector<StoredSurfaceNormal>& stored_surface_normals = lookup.first;
                if (stored_surface_normals.size() == 0)
                {
                    return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), true);
                }
                else
                {
                    // We get the "best" match surface normal given our entry direction
                    return std::pair<Eigen::Vector3d, bool>(GetBestSurfaceNormal(stored_surface_normals, direction), true);
                }
            }
            else
            {
                return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), false);
            }
        }

        inline std::pair<Eigen::Vector3d, bool> LookupSurfaceNormal(const int64_t x_index, const int64_t y_index, const int64_t z_index, const Eigen::Vector4d& direction) const
        {
            assert(initialized_);
            const std::pair<const std::vector<StoredSurfaceNormal>&, bool> lookup = surface_normal_grid_.GetImmutable(x_index, y_index, z_index);
            if (lookup.second)
            {
                const std::vector<StoredSurfaceNormal>& stored_surface_normals = lookup.first;
                if (stored_surface_normals.size() == 0)
                {
                    return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), true);
                }
                else
                {
                    // We get the "best" match surface normal given our entry direction
                    return std::pair<Eigen::Vector3d, bool>(GetBestSurfaceNormal(stored_surface_normals, direction), true);
                }
            }
            else
            {
                return std::pair<Eigen::Vector3d, bool>(Eigen::Vector3d(0.0, 0.0, 0.0), false);
            }
        }

        inline bool InsertSurfaceNormal(const double x, const double y, const double z, const Eigen::Vector3d& surface_normal, const Eigen::Vector3d& entry_direction)
        {
            assert(initialized_);
            const Eigen::Vector3d location(x, y, z);
            return InsertSurfaceNormal(location, surface_normal, entry_direction);
        }

        inline bool InsertSurfaceNormal(const Eigen::Vector3d& location, const Eigen::Vector3d& surface_normal, const Eigen::Vector3d& entry_direction)
        {
            assert(initialized_);
            const VoxelGrid::GRID_INDEX index = surface_normal_grid_.LocationToGridIndex3d(location);
            if (surface_normal_grid_.IndexInBounds(index))
            {
                return InsertSurfaceNormal(index, surface_normal, entry_direction);
            }
            else
            {
                return false;
            }
        }

        inline bool InsertSurfaceNormal(const VoxelGrid::GRID_INDEX& index, const Eigen::Vector3d& surface_normal, const Eigen::Vector3d& entry_direction)
        {
            assert(initialized_);
            return InsertSurfaceNormal(index.x, index.y, index.z, surface_normal, entry_direction);
        }

        inline bool InsertSurfaceNormal(const int64_t x_index, const int64_t y_index, const int64_t z_index, const Eigen::Vector3d& surface_normal, const Eigen::Vector3d& entry_direction)
        {
            assert(initialized_);
            std::pair<std::vector<StoredSurfaceNormal>&, bool> cell_query = surface_normal_grid_.GetMutable(x_index, y_index, z_index);
            if (cell_query.second)
            {
                std::vector<StoredSurfaceNormal>& cell_normals = cell_query.first;
                cell_normals.push_back(StoredSurfaceNormal(surface_normal, entry_direction));
                return true;
            }
            else
            {
                return false;
            }
        }

        inline bool ClearStoredSurfaceNormals(const double x, const double y, const double z)
        {
            assert(initialized_);
            const Eigen::Vector3d location(x, y, z);
            return ClearStoredSurfaceNormals(location);
        }

        inline bool ClearStoredSurfaceNormals(const Eigen::Vector3d& location)
        {
            assert(initialized_);
            const VoxelGrid::GRID_INDEX index = surface_normal_grid_.LocationToGridIndex3d(location);
            if (surface_normal_grid_.IndexInBounds(index))
            {
                return ClearStoredSurfaceNormals(index);
            }
            else
            {
                return false;
            }
        }

        inline bool ClearStoredSurfaceNormals(const VoxelGrid::GRID_INDEX& index)
        {
            assert(initialized_);
            return ClearStoredSurfaceNormals(index.x, index.y, index.z);
        }

        inline bool ClearStoredSurfaceNormals(const int64_t x_index, const int64_t y_index, const int64_t z_index)
        {
            assert(initialized_);
            std::pair<std::vector<StoredSurfaceNormal>&, bool> cell_query = surface_normal_grid_.GetMutable(x_index, y_index, z_index);
            if (cell_query.second)
            {
                std::vector<StoredSurfaceNormal>& cell_normals = cell_query.first;
                cell_normals.clear();
                return true;
            }
            else
            {
                return false;
            }
        }
    };

    struct SimulatorSolverParameters
    {
        double environment_collision_check_tolerance;
        double resolve_correction_step_scaling_decay_rate;
        double resolve_correction_initial_step_size;
        double resolve_correction_min_step_scaling;
        uint32_t max_resolver_iterations;
        uint32_t resolve_correction_step_scaling_decay_iterations;
        bool failed_resolves_end_motion;

        SimulatorSolverParameters()
        {
            environment_collision_check_tolerance = 0.001;
            resolve_correction_step_scaling_decay_rate = 0.5;
            resolve_correction_initial_step_size = 1.0;
            resolve_correction_min_step_scaling = 0.03125;
            max_resolver_iterations = 25;
            resolve_correction_step_scaling_decay_iterations = 5;
            failed_resolves_end_motion = true;
        }
    };

    template<typename DerivedRobotType, typename Configuration, typename RNG, typename ConfigAlloc=std::allocator<Configuration>>
    class SimpleParticleContactSimulator : public simple_simulator_interface::SimulatorInterface<Configuration, RNG, ConfigAlloc>
    {
    protected:

        typedef simple_robot_model_interface::SimpleRobotModelInterface<Configuration, ConfigAlloc> BaseRobotType;

        sdf_tools::TaggedObjectCollisionMapGrid environment_;
        sdf_tools::SignedDistanceField environment_sdf_;
        SurfaceNormalGrid surface_normals_grid_;
        std::map<uint32_t, uint32_t> convex_segment_counts_;
        bool initialized_;
        bool simulate_with_individual_jacobians_;
        double simulation_controller_frequency_;
        double simulation_controller_interval_;
        double contact_distance_threshold_;
        double resolution_distance_threshold_;
        SimulatorSolverParameters solver_config_;
        mutable std::vector<RNG> rngs_;
        mutable std::atomic<uint64_t> simulation_call_;
        mutable std::atomic<uint64_t> successful_resolves_;
        mutable std::atomic<uint64_t> unsuccessful_resolves_;
        mutable std::atomic<uint64_t> free_resolves_;
        mutable std::atomic<uint64_t> collision_resolves_;
        mutable std::atomic<uint64_t> fallback_resolves_;
        mutable std::atomic<uint64_t> unsuccessful_env_collision_resolves_;
        mutable std::atomic<uint64_t> unsuccessful_self_collision_resolves_;
        mutable std::atomic<uint64_t> recovered_unsuccessful_resolves_;

        static inline size_t GetNumOMPThreads()
        {
            #if defined(_OPENMP)
            size_t num_threads = 0;
            #pragma omp parallel
            {
                num_threads = (size_t)omp_get_num_threads();
            }
            return num_threads;
            #else
            return 1;
            #endif
        }

    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        SimpleParticleContactSimulator(const sdf_tools::TaggedObjectCollisionMapGrid& environment, const sdf_tools::SignedDistanceField& environment_sdf, const SurfaceNormalGrid& surface_normals_grid, const SimulatorSolverParameters& solver_config, const double simulation_controller_frequency, const bool simulate_with_individual_jacobians, const uint64_t prng_seed, const int32_t debug_level) : simple_simulator_interface::SimulatorInterface<Configuration, RNG, ConfigAlloc>(debug_level), environment_(environment), environment_sdf_(environment_sdf), surface_normals_grid_(surface_normals_grid)
        {
            simulate_with_individual_jacobians_ = simulate_with_individual_jacobians;
            contact_distance_threshold_ = 0.0;
            resolution_distance_threshold_ = 0.0;
            simulation_controller_frequency_ = std::abs(simulation_controller_frequency);
            simulation_controller_interval_ = 1.0 / simulation_controller_frequency;
            solver_config_ = solver_config;
            simulation_call_.store(0);
            // Prepare the default RNG
            RNG prng(prng_seed);
            // Temp seed distribution
            std::uniform_int_distribution<uint64_t> seed_dist(0, std::numeric_limits<uint64_t>::max());
            // Get the number of threads we're using
            const size_t num_threads = GetNumOMPThreads();
            // Prepare a number of PRNGs for each thread
            rngs_.clear();
            for (size_t tidx = 0; tidx < num_threads; tidx++)
            {
                rngs_.push_back(RNG(seed_dist(prng)));
            }
            ResetStatistics();
            initialized_ = true;
        }

        inline void ResetGenerators(const uint64_t prng_seed)
        {
            // Prepare the default RNG
            RNG prng(prng_seed);
            // Temp seed distribution
            std::uniform_int_distribution<uint64_t> seed_dist(0, std::numeric_limits<uint64_t>::max());
            // Get the number of threads we're using
            const uint32_t num_threads = GetNumOMPThreads();
            // Prepare a number of PRNGs for each thread
            rngs_.clear();
            for (uint32_t tidx = 0; tidx < num_threads; tidx++)
            {
                rngs_.push_back(PRNG(seed_dist(prng)));
            }
        }

        virtual RNG& GetRandomGenerator()
        {
        #if defined(_OPENMP)
            const size_t th_id = (size_t)omp_get_thread_num();
        #else
            const size_t th_id = 0;
        #endif
            return rngs_[th_id];
        }

        inline bool IsInitialized() const
        {
            return initialized_;
        }

        virtual std::map<std::string, double> GetStatistics() const
        {
            std::map<std::string, double> statistics;
            statistics["successful_resolves"] = (double)(successful_resolves_.load());
            statistics["unsuccessful_resolves"] = (double)(unsuccessful_resolves_.load());
            statistics["free_resolves"] = (double)(free_resolves_.load());
            statistics["collision_resolves"] = (double)(collision_resolves_.load());
            statistics["fallback_resolves"] = (double)(fallback_resolves_.load());
            statistics["unsuccessful_self_collision_resolves"] = (double)(unsuccessful_self_collision_resolves_.load());
            statistics["unsuccessful_env_collision_resolves"] = (double)(unsuccessful_env_collision_resolves_.load());
            statistics["recovered_unsuccessful_resolves"] = (double)(recovered_unsuccessful_resolves_.load());
            return statistics;
        }

        virtual void ResetStatistics()
        {
            successful_resolves_.store(0);
            unsuccessful_resolves_.store(0);
            free_resolves_.store(0);
            collision_resolves_.store(0);
            fallback_resolves_.store(0);
            unsuccessful_self_collision_resolves_.store(0);
            unsuccessful_env_collision_resolves_.store(0);
            recovered_unsuccessful_resolves_.store(0);
        }

        inline Eigen::Isometry3d GetOriginTransform() const
        {
            return environment_.GetOriginTransform();
        }

        virtual std::string GetFrame() const
        {
            return environment_.GetFrame();
        }

        inline double GetResolution() const
        {
            return environment_.GetResolution();
        }

        inline const sdf_tools::TaggedObjectCollisionMapGrid& GetEnvironment() const
        {
            return environment_;
        }

        inline sdf_tools::TaggedObjectCollisionMapGrid& GetMutableEnvironment()
        {
            return environment_;
        }

        inline const sdf_tools::SignedDistanceField& GetEnvironmentSDF() const
        {
            return environment_sdf_;
        }

        inline sdf_tools::SignedDistanceField& GetMutableEnvironmentSDF()
        {
            return environment_sdf_;
        }

        inline visualization_msgs::Marker ExportEnvironmentForDisplay(const float alpha=1.0f) const
        {
            return environment_.ExportForDisplay(alpha);
        }

        inline visualization_msgs::Marker ExportSDFForDisplay(const float alpha=1.0f) const
        {
            return environment_sdf_.ExportForDisplay(alpha);
        }

        virtual visualization_msgs::MarkerArray MakeEnvironmentDisplayRep() const
        {
            visualization_msgs::MarkerArray display_markers;
            visualization_msgs::Marker env_marker = environment_.ExportForDisplay();
            env_marker.id = 1;
            env_marker.ns = "sim_environment";
            display_markers.markers.push_back(env_marker);
            visualization_msgs::Marker components_marker = environment_.ExportConnectedComponentsForDisplay(false);
            components_marker.id = 1;
            components_marker.ns = "sim_environment_components";
            display_markers.markers.push_back(components_marker);
            visualization_msgs::Marker env_sdf_marker = environment_sdf_.ExportForDisplay(1.0f);
            env_sdf_marker.id = 1;
            env_sdf_marker.ns = "sim_environment_sdf";
            display_markers.markers.push_back(env_sdf_marker);
            // Draw all the convex segments for each object
            for (auto convex_segment_counts_itr = convex_segment_counts_.begin(); convex_segment_counts_itr != convex_segment_counts_.end(); ++convex_segment_counts_itr)
            {
                const uint32_t object_id = convex_segment_counts_itr->first;
                const uint32_t convex_segment_count = convex_segment_counts_itr->second;
                for (uint32_t convex_segment = 1; convex_segment <= convex_segment_count; convex_segment++)
                {
                    const visualization_msgs::Marker segment_marker = environment_.ExportConvexSegmentForDisplay(object_id, convex_segment);
                    display_markers.markers.push_back(segment_marker);
                }
            }
            return display_markers;
        }

        inline visualization_msgs::MarkerArray MakeConfigurationDisplayRepMixed(const std::shared_ptr<BaseRobotType>& immutable_robot, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const Configuration& configuration, const std_msgs::ColorRGBA& color, const int32_t starting_index, const std::string& config_marker_ns) const
        {
            // Perform FK on the current config
            std::shared_ptr<BaseRobotType> working_robot(immutable_robot->Clone());
            working_robot->SetPosition(configuration);
            // Now, go through the links and points of the robot for collision checking
            visualization_msgs::MarkerArray configuration_markers;
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the transform of the current link
                const Eigen::Isometry3d link_transform = working_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& raw_link_sphere = link_points[point_idx];
                    const double sphere_radius = (link_geometry_type == simple_robot_models::PointSphereGeometry::SPHERES) ? raw_link_sphere(3) : (this->GetResolution() * 0.5);
                    const Eigen::Vector4d link_relative_sphere_origin(raw_link_sphere(0), raw_link_sphere(1), raw_link_sphere(2), 1.0);
                    const Eigen::Vector4d environment_relative_sphere_origin = link_transform * link_relative_sphere_origin;
                    // Make the marker for the current sphere
                    std_msgs::ColorRGBA real_color = color;
                    visualization_msgs::Marker configuration_marker;
                    configuration_marker.action = visualization_msgs::Marker::ADD;
                    configuration_marker.ns = config_marker_ns;
                    configuration_marker.id = starting_index + (uint32_t)configuration_markers.markers.size();
                    configuration_marker.frame_locked = false;
                    configuration_marker.lifetime = ros::Duration(0.0);
                    configuration_marker.type = visualization_msgs::Marker::SPHERE;
                    configuration_marker.header.frame_id = this->GetFrame();
                    configuration_marker.scale.x = sphere_radius * 2.0;
                    configuration_marker.scale.y = sphere_radius * 2.0;
                    configuration_marker.scale.z = sphere_radius * 2.0;
                    configuration_marker.pose.position = EigenHelpersConversions::EigenVector4dToGeometryPoint(environment_relative_sphere_origin);
                    configuration_marker.pose.orientation = EigenHelpersConversions::EigenQuaterniondToGeometryQuaternion(Eigen::Quaterniond::Identity());
                    configuration_marker.color = real_color;
                    // Store it
                    configuration_markers.markers.push_back(configuration_marker);
                }
            }
            return configuration_markers;
        }

        inline visualization_msgs::MarkerArray MakeConfigurationDisplayRepPoints(const std::shared_ptr<BaseRobotType>& immutable_robot, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const Configuration& configuration, const std_msgs::ColorRGBA& color, const int32_t starting_index, const std::string& config_marker_ns) const
        {
            std::shared_ptr<BaseRobotType> working_robot(immutable_robot->Clone());
            std_msgs::ColorRGBA real_color = color;
            visualization_msgs::Marker configuration_marker;
            configuration_marker.action = visualization_msgs::Marker::ADD;
            configuration_marker.ns = config_marker_ns;
            configuration_marker.id = starting_index;
            configuration_marker.frame_locked = false;
            configuration_marker.lifetime = ros::Duration(0.0);
            configuration_marker.type = visualization_msgs::Marker::SPHERE_LIST;
            configuration_marker.header.frame_id = this->GetFrame();
            configuration_marker.scale.x = this->GetResolution();
            configuration_marker.scale.y = this->GetResolution();
            configuration_marker.scale.z = this->GetResolution();
            const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
            configuration_marker.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
            configuration_marker.color = real_color;
            // Make the individual points
            // Update the position of the robot
            working_robot->SetPosition(configuration);
            // Now, go through the links and points of the robot for collision checking
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                assert(link_geometry_type == simple_robot_models::PointSphereGeometry::POINTS);
                // Get the transform of the current link
                const Eigen::Isometry3d link_transform = working_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    const geometry_msgs::Point marker_point = EigenHelpersConversions::EigenVector4dToGeometryPoint(environment_relative_point);
                    configuration_marker.points.push_back(marker_point);
                    if (link_relative_point.norm() == 0.0)
                    {
                        std_msgs::ColorRGBA black_color;
                        black_color.r = 0.0f;
                        black_color.g = 0.0f;
                        black_color.b = 0.0f;
                        black_color.a = 1.0f;
                        configuration_marker.colors.push_back(black_color);
                    }
                    else
                    {
                        configuration_marker.colors.push_back(real_color);
                    }
                }
            }
            visualization_msgs::MarkerArray configuration_markers;
            configuration_markers.markers = {configuration_marker};
            return configuration_markers;
        }

        virtual visualization_msgs::MarkerArray MakeConfigurationDisplayRep(const std::shared_ptr<BaseRobotType>& immutable_robot, const Configuration& configuration, const std_msgs::ColorRGBA& color, const int32_t starting_index, const std::string& config_marker_ns) const
        {
            bool points_only = true;
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(immutable_robot.get())->GetLinkGeometries();
            // Now, go through the links and points of the robot for collision checking
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                if (link_geometry_type != simple_robot_models::PointSphereGeometry::POINTS)
                {
                    points_only = false;
                }
            }
            if (points_only)
            {
                return MakeConfigurationDisplayRepPoints(immutable_robot, robot_link_geometries, configuration, color, starting_index, config_marker_ns);
            }
            else
            {
                return MakeConfigurationDisplayRepMixed(immutable_robot, robot_link_geometries, configuration, color, starting_index, config_marker_ns);
            }
        }

        virtual visualization_msgs::MarkerArray MakeControlInputDisplayRep(const std::shared_ptr<BaseRobotType>& immutable_robot, const Configuration& configuration, const Eigen::VectorXd& control_input, const std_msgs::ColorRGBA& color, const int32_t starting_index, const std::string& control_input_marker_ns) const
        {
            std::shared_ptr<BaseRobotType> robot(immutable_robot->Clone());
            std_msgs::ColorRGBA real_color = color;
            visualization_msgs::Marker control_input_marker;
            control_input_marker.action = visualization_msgs::Marker::ADD;
            control_input_marker.ns = control_input_marker_ns;
            control_input_marker.id = starting_index;
            control_input_marker.frame_locked = false;
            control_input_marker.lifetime = ros::Duration(0.0);
            control_input_marker.type = visualization_msgs::Marker::LINE_LIST;
            control_input_marker.header.frame_id = this->GetFrame();
            control_input_marker.scale.x = this->GetResolution() * 0.5;
            control_input_marker.scale.y = this->GetResolution() * 0.5;
            control_input_marker.scale.z = this->GetResolution() * 0.5;
            const Eigen::Isometry3d base_transform = Eigen::Isometry3d::Identity();
            control_input_marker.pose = EigenHelpersConversions::EigenIsometry3dToGeometryPose(base_transform);
            control_input_marker.color = real_color;
            // Make the indivudal points
            // Get the list of link name + link points for all the links of the robot
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(immutable_robot.get())->GetLinkGeometries();
            // Now, go through the links and points of the robot for collision checking
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the current transform
                // Update the position of the robot
                robot->SetPosition(configuration);
                // Get the transform of the current link
                const Eigen::Isometry3d current_link_transform = robot->GetLinkTransform(link_name);
                // Apply the control input
                static_cast<DerivedRobotType*>(robot.get())->ApplyControlInput(control_input);
                // Get the transform of the current link
                const Eigen::Isometry3d current_plus_control_link_transform = robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_current_point = current_link_transform * link_relative_point;
                    const Eigen::Vector4d environment_relative_current_plus_control_point = current_plus_control_link_transform * link_relative_point;
                    const geometry_msgs::Point current_marker_point = EigenHelpersConversions::EigenVector4dToGeometryPoint(environment_relative_current_point);
                    const geometry_msgs::Point current_plus_control_marker_point = EigenHelpersConversions::EigenVector4dToGeometryPoint(environment_relative_current_plus_control_point);
                    control_input_marker.points.push_back(current_marker_point);
                    control_input_marker.points.push_back(current_plus_control_marker_point);
                    control_input_marker.colors.push_back(real_color);
                    control_input_marker.colors.push_back(real_color);
                }
            }
            visualization_msgs::MarkerArray control_input_markers;
            control_input_markers.markers = {control_input_marker};
            return control_input_markers;
        }

        virtual Eigen::Vector4d Get3dPointForConfig(const std::shared_ptr<BaseRobotType>& immutable_robot, const Configuration& config) const
        {
            std::shared_ptr<BaseRobotType> robot(immutable_robot->Clone());
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(immutable_robot.get())->GetLinkGeometries();
            robot->SetPosition(config);
            const std::string& link_name = robot_link_geometries.back().first;
            const Eigen::Isometry3d link_transform = robot->GetLinkTransform(link_name);
            const Eigen::Vector4d link_relative_point(0.0, 0.0, 0.0, 1.0);
            const Eigen::Vector4d config_point = link_transform * link_relative_point;
            return config_point;
        }

        virtual std::vector<std::pair<Configuration, bool>> ForwardSimulateRobots(const std::shared_ptr<BaseRobotType>& immutable_robot, const std::vector<Configuration, ConfigAlloc>& start_positions, const std::vector<Configuration, ConfigAlloc>& target_positions, const double forward_simulation_time, const double simulation_shortcut_distance, const bool allow_contacts, const std::function<void(const visualization_msgs::MarkerArray&)>& display_fn) const
        {
            if (start_positions.size() > 0)
            {
                assert((target_positions.size() == 1) || (target_positions.size() == start_positions.size()));
            }
            std::vector<std::pair<Configuration, bool>> propagated_points(start_positions.size());
            #pragma omp parallel for
            for (size_t idx = 0; idx < start_positions.size(); idx++)
            {
                const Configuration& initial_particle = start_positions[idx];
                const Configuration& target_position = (target_positions.size() == start_positions.size()) ? target_positions[idx] : target_positions.front();
                simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc> trace;
                propagated_points[idx] = ForwardSimulateRobot(immutable_robot, initial_particle, target_position, forward_simulation_time, simulation_shortcut_distance, allow_contacts, trace, false, display_fn);
            }
            return propagated_points;
        }

        virtual std::pair<Configuration, bool> ForwardSimulateRobot(const std::shared_ptr<BaseRobotType>& immutable_robot, const Configuration& start_position, const Configuration& target_position, const double forward_simulation_time, const double simulation_shortcut_distance, const bool allow_contacts, simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc>& trace, const bool enable_tracing, const std::function<void(const visualization_msgs::MarkerArray&)>& display_fn) const
        {
            std::shared_ptr<BaseRobotType> robot(immutable_robot->Clone());
            static_cast<DerivedRobotType*>(robot.get())->ResetPosition(start_position);
            return ForwardSimulateMutableRobot(robot, target_position, forward_simulation_time, simulation_shortcut_distance, allow_contacts, trace, enable_tracing, display_fn);
        }

        virtual std::pair<Configuration, bool> ForwardSimulateMutableRobot(const std::shared_ptr<BaseRobotType>& robot, const Configuration& target_position, const double forward_simulation_time, const double simulation_shortcut_distance, const bool allow_contacts, simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc>& trace, const bool enable_tracing, const std::function<void(const visualization_msgs::MarkerArray&)>& display_fn) const
        {
        #if defined(_OPENMP)
            const size_t th_id = (size_t)omp_get_thread_num();
        #else
            const size_t th_id = 0;
        #endif
            RNG& rng = rngs_[th_id];
            simulation_call_.fetch_add(1u);
            const uint64_t call_number = simulation_call_.load();
            const Configuration start_position = robot->GetPosition();
            // Forward simulate for the provided number of steps
            bool collided = false;
            const uint32_t forward_simulation_steps = std::max((uint32_t)(forward_simulation_time * simulation_controller_frequency_), 1u);
            bool any_resolve_failed = false;
            if (this->debug_level_ >= 1)
            {
                const std::string msg = "[" + std::to_string(call_number) + "] Starting simulation:\nStart: " + PrettyPrint::PrettyPrint(start_position) + "\nTarget: " + PrettyPrint::PrettyPrint(target_position);
                std::cout << msg << std::endl;
            }
            for (uint32_t step = 0; step < forward_simulation_steps; step++)
            {
                // Step forward via the simulator
                // Have robot compute next control input first
                // Then, in a second function *not* in a callback, apply that control input
                const Eigen::VectorXd control_action = static_cast<DerivedRobotType*>(robot.get())->GenerateControlAction(target_position, simulation_controller_interval_);
                const std::pair<Configuration, std::pair<bool, bool>> result = ResolveForwardSimulation(robot, control_action, simulation_controller_interval_, rng, simulate_with_individual_jacobians_, allow_contacts, trace, enable_tracing, call_number, display_fn);
                const Configuration& resolved_configuration = result.first;
                const bool resolve_collided = result.second.first;
                const bool current_resolve_failed = result.second.second;
                if ((allow_contacts == true) || (resolve_collided == false))
                {
                    robot->SetPosition(resolved_configuration);
                    // Check if we've collided with the environment
                    if (resolve_collided)
                    {
                        collided = true;
                    }
                    // Check for stats
                    if (current_resolve_failed)
                    {
                        if (solver_config_.failed_resolves_end_motion)
                        {
                            break;
                        }
                        any_resolve_failed = true;
                    }
                    else
                    {
                        if (any_resolve_failed)
                        {
                            recovered_unsuccessful_resolves_.fetch_add(1);
                        }
                    }
                    // Last, but not least, check if we've gotten close enough the target state to short-circut the simulation
                    const double target_distance = robot->ComputeConfigurationDistance(target_position);
                    if (target_distance < simulation_shortcut_distance)
                    {
                        break;
                    }
                }
                else
                {
                    assert(resolve_collided == true);
                    // If we don't allow contacts, we don't update the position and we stop simulating
                    break;
                }
            }
            // Return the ending position of the robot and if it has collided during simulation
            const Configuration reached_position = robot->GetPosition();
            if (this->debug_level_ >= 1)
            {
                const std::string msg = "[" + std::to_string(call_number) + "] Forward simulated in " + std::to_string(forward_simulation_steps) + " steps from\nStart: " + PrettyPrint::PrettyPrint(start_position) + "\nTarget: " + PrettyPrint::PrettyPrint(target_position) + "\nReached: " + PrettyPrint::PrettyPrint(reached_position);
                std::cout << msg << std::endl;
            }
            return std::pair<Configuration, bool>(reached_position, collided);
        }

        inline bool CheckEnvironmentCollision(const std::shared_ptr<BaseRobotType>& robot, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const double collision_threshold) const
        {
            const double real_collision_threshold = collision_threshold - (solver_config_.environment_collision_check_tolerance * this->environment_sdf_.GetResolution());
            // Now, go through the links and points of the robot for collision checking
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future to check radius on sphere-based models
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the transform of the current link
                const Eigen::Isometry3d link_transform = robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    const std::pair<float, bool> sdf_check = this->environment_sdf_.GetSafe4d(environment_relative_point);
                    //const std::pair<double, bool> sdf_check = this->environment_sdf_.EstimateDistance(environment_relative_point);
                    if (sdf_check.second == false)
                    {
                        if (this->debug_level_ >= 1)
                        {
                            const std::string msg = "Point at " + PrettyPrint::PrettyPrint(environment_relative_point) + " out of bounds";
                            std::cerr << msg << std::endl;
                        }
#ifdef ASSERT_ON_OUT_OF_BOUNDS
                        assert(false);
#endif
                    }
                    // We only work with points in collision
                    if (sdf_check.first < real_collision_threshold)
                    {
                        if (sdf_check.first < (real_collision_threshold - this->environment_sdf_.GetResolution()))
                        {
                            if (this->debug_level_ >= 25)
                            {
                                std::cout << "Point at " << PrettyPrint::PrettyPrint(environment_relative_point) << " in collision with SDF distance " << sdf_check.first << " and threshold " << real_collision_threshold;
                            }
                            return true;
                        }
                        else
                        {
                            const double estimated_distance = this->environment_sdf_.EstimateDistance4d(environment_relative_point).first;
                            if (estimated_distance < real_collision_threshold)
                            {
                                if (this->debug_level_ >= 25)
                                {
                                    std::cout << "Point at " << PrettyPrint::PrettyPrint(environment_relative_point) << " in collision with SDF distance " << sdf_check.first << " and threshold " << real_collision_threshold;
                                }
                                return true;
                            }
                        }
                    }
                }
            }
            return false;
        }

        inline std::map<std::pair<size_t, size_t>, Eigen::Vector3d> ExtractSelfCollidingPoints(const std::shared_ptr<BaseRobotType>& previous_robot, const std::shared_ptr<BaseRobotType>& current_robot, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const std::vector<std::pair<size_t, size_t>>& candidate_points, const std::map<size_t, double>& link_masses, const double time_interval) const
        {
            if (candidate_points.size() > 1)
            {
                // Now, we separate the points by link
                std::map<size_t, std::vector<size_t>> point_self_collision_check_map;
                for (size_t idx = 0; idx < candidate_points.size(); idx++)
                {
                    const std::pair<size_t, size_t>& point = candidate_points[idx];
                    point_self_collision_check_map[point.first].push_back(point.second);
                }
                //std::cout << "Considering " << point_self_collision_check_map.size() << " separate links with self-colliding points" << std::endl;
                // Let's see how many links we have - we only care if multiple links are involved
                if (point_self_collision_check_map.size() >= 2)
                {
                    // For each link, figure out which *other* links it is colliding with
                    std::map<size_t, std::vector<size_t>> link_collisions;
                    for (auto fitr = point_self_collision_check_map.begin(); fitr != point_self_collision_check_map.end(); ++fitr)
                    {
                        for (auto sitr = point_self_collision_check_map.begin(); sitr != point_self_collision_check_map.end(); ++sitr)
                        {
                            if (fitr != sitr)
                            {
                                const size_t fitr_link = fitr->first;
                                const size_t sitr_link = sitr->first;
                                const bool self_collision_allowed = static_cast<DerivedRobotType*>(current_robot.get())->CheckIfSelfCollisionAllowed(fitr_link, sitr_link);
                                if (self_collision_allowed == false)
                                {
                                    //const std::string msg = "Self collisions not allowed between " + std::to_string(fitr_link) + " and " + std::to_string(sitr_link);
                                    //std::cout << msg << std::endl;
                                    link_collisions[fitr_link].push_back(sitr_link);
                                }
                            }
                        }
                    }
                    if (link_collisions.size() < 2)
                    {
                        return std::map<std::pair<size_t, size_t>, Eigen::Vector3d>();
                    }
                    //std::cout << "Self collisions: " << PrettyPrint::PrettyPrint(link_collisions) << std::endl;
                    // We go through each link and compute an "input momentum" vector that reflects the contribution of the particular link to the collision
                    // We can assume that point motion has occurred over unit time, so motion = velocity, and that each point has unit mass, so momentum = velocity.
                    // Thus, the momentum = motion for each particle
                    // !!! FIX TO INCLUDE LINK & FURTHER LINK(S) MASS SO LOWER LINKS MOVE LESS !!!
                    const double time_multiplier = 1.0 / time_interval;
                    std::map<size_t, Eigen::Vector4d> link_momentum_vectors;
                    for (auto link_itr = point_self_collision_check_map.begin(); link_itr != point_self_collision_check_map.end(); ++link_itr)
                    {
                        const size_t link_idx = link_itr->first;
                        // Skip links we already filtered out due to allowed self collision
                        if (link_collisions.find(link_idx) != link_collisions.end())
                        {
                            const std::string& link_name = robot_link_geometries[link_itr->first].first;
                            const Eigen::Isometry3d previous_link_transform = previous_robot->GetLinkTransform(link_name);
                            const Eigen::Isometry3d current_link_transform = current_robot->GetLinkTransform(link_name);
                            const std::vector<size_t>& link_points = link_itr->second;
                            Eigen::Vector4d link_momentum_vector(0.0, 0.0, 0.0, 0.0);
                            for (size_t idx = 0; idx < link_points.size(); idx++)
                            {
                                const size_t link_point = link_points[idx];
                                const Eigen::Vector4d& link_relative_point = (*robot_link_geometries[link_idx].second.Geometry())[link_point];
                                const Eigen::Vector4d previous_point_location = previous_link_transform * link_relative_point;
                                const Eigen::Vector4d current_point_location = current_link_transform * link_relative_point;
                                const Eigen::Vector4d point_motion = current_point_location - previous_point_location;
                                const Eigen::Vector4d point_velocity = point_motion * time_multiplier; // point motion/interval * interval/sec
                                if (point_velocity.norm() <= std::numeric_limits<double>::epsilon())
                                {
                                    //const std::string msg = "Point motion would be zero (link " + std::to_string(link_idx) + ", point " + std::to_string(link_point) + ")\nPrevious location: " + PrettyPrint::PrettyPrint(previous_point_location) + "\nCurrent location: " + PrettyPrint::PrettyPrint(current_point_location);
                                    //std::cout << msg << std::endl;
                                }
                                link_momentum_vector = link_momentum_vector + point_velocity;
                            }
                            link_momentum_vectors[link_idx] = link_momentum_vector;
                        }
                    }
                    //std::cout << "Link momentum vectors:\n" << PrettyPrint::PrettyPrint(link_momentum_vectors) << std::endl;
                    // Store the corrections we compute
                    std::map<std::pair<size_t, size_t>, Eigen::Vector3d> self_colliding_points_with_corrections;
                    // Now, for each link, we compute a correction for each colliding point on the link
                    for (auto link_itr = link_collisions.begin(); link_itr != link_collisions.end(); ++link_itr)
                    {
                        const size_t link_idx = link_itr->first;
                        const std::string& link_name = robot_link_geometries[link_idx].first;
                        const Eigen::Isometry3d previous_link_transform = previous_robot->GetLinkTransform(link_name);
                        const Eigen::Vector4d& link_point = (*robot_link_geometries[link_idx].second.Geometry())[point_self_collision_check_map[link_idx].front()];
                        const Eigen::Vector4d link_point_location = previous_link_transform * link_point;
                        const std::vector<size_t>& colliding_links = link_itr->second;
                        const auto link_momentum_vector_query = link_momentum_vectors.find(link_idx);
                        assert(link_momentum_vector_query != link_momentum_vectors.end());
                        const Eigen::Vector4d link_momentum_vector = link_momentum_vector_query->second;
                        const Eigen::Vector4d link_velocity = link_momentum_vector / (double)point_self_collision_check_map[link_idx].size();
                        // We compute a whole-link correction
                        // For the purposes of simulation, we assume an elastic collision - i.e. momentum must be conserved
                        Eigen::MatrixXd contact_matrix = Eigen::MatrixXd::Zero((ssize_t)((colliding_links.size() + 1) * 3), (ssize_t)(colliding_links.size() * 3));
                        // For each link, fill in the contact matrix
                        for (int64_t link = 1; link <= (int64_t)colliding_links.size(); link++)
                        {
                            int64_t collision_number = link - 1;
                            // Our current link gets -I
                            contact_matrix.block<3, 3>(0, (collision_number * 3)) = (Eigen::MatrixXd::Identity(3, 3) * -1.0);
                            // The other link gets +I
                            contact_matrix.block<3, 3>((link * 3), (collision_number * 3)) = Eigen::MatrixXd::Identity(3, 3);
                        }
                        //std::cout << "Contact matrix:\n" << PrettyPrint::PrettyPrint(contact_matrix) << std::endl;
                        // Generate the contact normal matrix
                        Eigen::MatrixXd contact_normal_matrix = Eigen::MatrixXd::Zero((ssize_t)(colliding_links.size() * 3), (ssize_t)(colliding_links.size()));
                        for (int64_t collision = 0; collision < (int64_t)colliding_links.size(); collision++)
                        {
                            const size_t other_link_idx = colliding_links[(size_t)collision];
                            const std::string& other_link_name = robot_link_geometries[other_link_idx].first;
                            const Eigen::Isometry3d previous_other_link_transform = previous_robot->GetLinkTransform(other_link_name);
                            const Eigen::Vector4d& other_link_point = (*robot_link_geometries[other_link_idx].second.Geometry())[point_self_collision_check_map[other_link_idx].front()];
                            const Eigen::Vector4d other_link_point_location = previous_other_link_transform * other_link_point;
                            // Compute the contact normal
                            //const Eigen::Vector3d other_link_velocity = link_momentum_vectors[other_link_idx] / (double)point_self_collision_check_map[other_link_idx].size();
                            //const Eigen::Vector3d current_link_position = link_velocity * -1.0;
                            //const Eigen::Vector3d current_other_link_position = other_link_velocity * -1.0;
                            //const Eigen::Vector3d contact_direction = current_other_link_position - current_link_position;
                            const Eigen::Vector4d contact_direction = other_link_point_location - link_point_location;
                            const Eigen::Vector4d contact_normal = EigenHelpers::SafeNormal(contact_direction);
                            //const std::string msg = "Contact normal: " + PrettyPrint::PrettyPrint(contact_normal) + "\nCurrent link position: " + PrettyPrint::PrettyPrint(link_point_location) + "\nOther link position: " + PrettyPrint::PrettyPrint(other_link_point_location);
                            //std::cout << msg << std::endl;
                            contact_normal_matrix.block<3, 1>((collision * 3), collision) = contact_normal.block<3, 1>(0, 0);
                        }
                        //std::cout << "Contact normal matrix:\n" << PrettyPrint::PrettyPrint(contact_normal_matrix) << std::endl;
                        // Generate the mass matrix
                        Eigen::MatrixXd mass_matrix = Eigen::MatrixXd::Zero((ssize_t)((colliding_links.size() + 1) * 3), (ssize_t)((colliding_links.size() + 1) * 3));
                        // Add the mass of our link
                        const double link_mass = link_masses.find(link_idx)->second; // (double)point_self_collision_check_map[link_idx].size();
                        mass_matrix.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3) * link_mass;
                        // Add the mass of the other links
                        for (int64_t link = 1; link <= (int64_t)colliding_links.size(); link++)
                        {
                            const size_t other_link_idx = colliding_links[(size_t)(link - 1)];
                            const double other_link_mass = link_masses.find(other_link_idx)->second; // (double)point_self_collision_check_map[other_link_idx].size();
                            mass_matrix.block<3, 3>((link * 3), (link * 3)) = Eigen::MatrixXd::Identity(3, 3) * other_link_mass;
                        }
                        //std::cout << "Mass matrix:\n" << PrettyPrint::PrettyPrint(mass_matrix) << std::endl;
                        // Generate the velocity matrix
                        Eigen::MatrixXd velocity_matrix = Eigen::MatrixXd::Zero((ssize_t)((colliding_links.size() + 1) * 3), (ssize_t)1);
                        velocity_matrix.block<3, 1>(0, 0) = link_velocity.block<3, 1>(0, 0);
                        for (int64_t link = 1; link <= (int64_t)colliding_links.size(); link++)
                        {
                            const size_t other_link_idx = colliding_links[(size_t)(link - 1)];
                            const Eigen::Vector4d other_link_velocity = link_momentum_vectors[other_link_idx] / (double)point_self_collision_check_map[other_link_idx].size();
                            velocity_matrix.block<3, 1>((link * 3), 0) = other_link_velocity.block<3, 1>(0, 0);
                        }
                        //std::cout << "Velocity matrix:\n" << PrettyPrint::PrettyPrint(velocity_matrix) << std::endl;
                        // Compute the impulse corrections
                        // Yes, this is ugly. This is to suppress a warning on type conversion related to Eigen operations
                        #pragma GCC diagnostic push
                        #pragma GCC diagnostic ignored "-Wconversion"
                        const Eigen::MatrixXd impulses = (contact_normal_matrix.transpose() * contact_matrix.transpose() * mass_matrix.inverse() * contact_matrix * contact_normal_matrix).inverse() * contact_normal_matrix.transpose() * contact_matrix.transpose() * velocity_matrix;
                        //std::cout << "Impulses:\n" << PrettyPrint::PrettyPrint(impulses) << std::endl;
                        // Compute the new velocities
                        const Eigen::MatrixXd velocity_delta = (mass_matrix.inverse() * contact_matrix * contact_normal_matrix * impulses) * -1.0;
                        //std::cout << "New velocities:\n" << velocity_delta << std::endl;
                        #pragma GCC diagnostic pop
                        // Extract the correction just for our current link
                        const Eigen::Vector3d link_correction_velocity = velocity_delta.block<3, 1>(0, 0);
                        // We then distribute that correction over the points on that link that have contributed to the collision
                        const std::vector<size_t>& link_points = point_self_collision_check_map[link_idx];
                        for (size_t idx = 0; idx < link_points.size(); idx++)
                        {
                            const size_t point_idx = link_points[idx];
                            const std::pair<size_t, size_t> point_id(link_idx, point_idx);
                            //std::cout << "Link correction velocity: " << PrettyPrint::PrettyPrint(link_correction_velocity) << std::endl;
                            const Eigen::Vector3d point_correction = link_correction_velocity / (double)(link_points.size());
                            assert(std::isnan(point_correction.x()) == false);
                            assert(std::isnan(point_correction.y()) == false);
                            assert(std::isnan(point_correction.z()) == false);
                            //std::cout << "Correction (new): " << PrettyPrint::PrettyPrint(point_id) << " - " << PrettyPrint::PrettyPrint(point_correction) << std::endl;
                            self_colliding_points_with_corrections[point_id] = point_correction;
                        }
                    }
                    return self_colliding_points_with_corrections;
                }
                // One link cannot be self-colliding
                else
                {
                    return std::map<std::pair<size_t, size_t>, Eigen::Vector3d>();
                }
            }
            // One point cannot be self-colliding
            else
            {
                return std::map<std::pair<size_t, size_t>, Eigen::Vector3d>();
            }
        }

        inline std::vector<int64_t> LocationToExtendedGridIndex(const Eigen::Vector4d& location, const double extended_grid_resolution) const
        {
            assert(initialized_);
            const Eigen::Vector4d point_in_grid_frame = this->environment_.GetInverseOriginTransform() * location;
            const int64_t x_cell = (int64_t)(point_in_grid_frame(0) / extended_grid_resolution);
            const int64_t y_cell = (int64_t)(point_in_grid_frame(1) / extended_grid_resolution);
            const int64_t z_cell = (int64_t)(point_in_grid_frame(2) / extended_grid_resolution);
            return std::vector<int64_t>{x_cell, y_cell, z_cell};
        }

        inline std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d> CollectSelfCollisions(const std::shared_ptr<BaseRobotType>& previous_robot, const std::shared_ptr<BaseRobotType>& current_robot, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const double time_interval) const
        {
            // Note that robots with only one link *cannot* self-collide!
            if (robot_link_geometries.size() == 1)
            {
                return std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>();
            }
            else if (robot_link_geometries.size() == 2)
            {
                // If the robot is only two links, and self-collision between them is allowed, we can avoid checks
                if (static_cast<DerivedRobotType*>(current_robot.get())->CheckIfSelfCollisionAllowed(0, 1))
                {
                    return std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>();
                }
            }
            // We use a hashtable to detect self-collisions
            std::unordered_map<VoxelGrid::GRID_INDEX, std::vector<std::pair<size_t, size_t>>> self_collision_check_map;
            // Now, go through the links and points of the robot for collision checking
            bool any_candidate_self_collisions = false;
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future for radius information
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the transform of the current link
                const Eigen::Isometry3d link_transform = current_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    // Get the corresponding index
                    const std::vector<int64_t> index = LocationToExtendedGridIndex(environment_relative_point, this->environment_.GetResolution());
                    assert(index.size() == 3);
                    VoxelGrid::GRID_INDEX point_index(index[0], index[1], index[2]);
                    // Insert the index into the map
                    std::vector<std::pair<size_t, size_t>>& map_cell = self_collision_check_map[point_index];
                    if (map_cell.size() > 1)
                    {
                        any_candidate_self_collisions = true;
                    }
                    else if (map_cell.size() == 1)
                    {
                        const std::pair<size_t, size_t>& current = map_cell[0];
                        if (current.first != link_idx)
                        {
                            any_candidate_self_collisions = true;
                        }
                    }
                    map_cell.push_back(std::pair<size_t, size_t>(link_idx, point_idx));
                }
            }
            if (any_candidate_self_collisions == false)
            {
                return std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>();
            }
            // Compute approximate link masses (in reverse order, since each link's mass is its mass + mass of all further links
            std::map<size_t, double> link_masses;
            double previous_link_masses = 0.0;
            for (int64_t link_idx = ((int64_t)robot_link_geometries.size() - 1); link_idx >= 0; link_idx--)
            {
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future for radius information
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                const double link_mass = (double)(link_points.size());
                link_masses[(size_t)link_idx] = link_mass + previous_link_masses;
                previous_link_masses += link_mass;
            }
            // Now, we go through the map and see if any points overlap
            // Store "true" self-collisions
            std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d> self_collisions;
            for (auto itr = self_collision_check_map.begin(); itr != self_collision_check_map.end(); ++itr)
            {
                //const VoxelGrid::GRID_INDEX& location = itr->first;
                const std::vector<std::pair<size_t, size_t>>& candidate_points = itr->second;
                //std::cout << "Candidate points: " << PrettyPrint::PrettyPrint(candidate_points) << std::endl;
                const std::map<std::pair<size_t, size_t>, Eigen::Vector3d> self_colliding_points = ExtractSelfCollidingPoints(previous_robot, current_robot, robot_link_geometries, candidate_points, link_masses, time_interval);
                //std::cout << "Extracted points: " << PrettyPrint::PrettyPrint(self_colliding_points) << std::endl;
                for (auto scpitr = self_colliding_points.begin(); scpitr != self_colliding_points.end(); ++scpitr)
                {
                    const std::pair<size_t, size_t>& self_colliding_point = scpitr->first;
                    const Eigen::Vector3d& correction = scpitr->second;
                    self_collisions[self_colliding_point] = correction;
                }
            }
            // If we haven't already returned, we are self-collision-free
            return self_collisions;
        }

        inline bool CheckPointsForSelfCollision(const std::shared_ptr<BaseRobotType>& current_robot, const std::vector<std::pair<size_t, size_t>>& candidate_points) const
        {
            if (candidate_points.size() > 1)
            {
                // Now, we separate the points by link
                std::map<size_t, std::vector<size_t>> point_self_collision_check_map;
                for (size_t idx = 0; idx < candidate_points.size(); idx++)
                {
                    const std::pair<size_t, size_t>& point = candidate_points[idx];
                    point_self_collision_check_map[point.first].push_back(point.second);
                }
                //std::cout << "Considering " << point_self_collision_check_map.size() << " separate links with self-colliding points" << std::endl;
                // Let's see how many links we have - we only care if multiple links are involved
                if (point_self_collision_check_map.size() >= 2)
                {
                    // For each link, figure out which *other* links it is colliding with
                    for (auto fitr = point_self_collision_check_map.begin(); fitr != point_self_collision_check_map.end(); ++fitr)
                    {
                        for (auto sitr = point_self_collision_check_map.begin(); sitr != point_self_collision_check_map.end(); ++sitr)
                        {
                            if (fitr != sitr)
                            {
                                const size_t fitr_link = fitr->first;
                                const size_t sitr_link = sitr->first;
                                const bool self_collision_allowed = static_cast<DerivedRobotType*>(current_robot.get())->CheckIfSelfCollisionAllowed(fitr_link, sitr_link);
                                if (self_collision_allowed == false)
                                {
                                    return true;
                                }
                            }
                        }
                    }
                    return false;
                }
                // One link cannot be self-colliding
                else
                {
                    return false;
                }
            }
            // One point cannot be self-colliding
            else
            {
                return false;
            }
        }

        inline bool CheckSelfCollisions(const std::shared_ptr<BaseRobotType>& current_robot, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const double check_resolution) const
        {
            // Note that robots with only one link *cannot* self-collide!
            if (robot_link_geometries.size() == 1)
            {
                return false;
            }
            else if (robot_link_geometries.size() == 2)
            {
                // If the robot is only two links, and self-collision between them is allowed, we can avoid checks
                if (static_cast<DerivedRobotType*>(current_robot.get())->CheckIfSelfCollisionAllowed(0, 1))
                {
                    return false;
                }
            }
            // We use a hashtable to detect self-collisions
            std::unordered_map<VoxelGrid::GRID_INDEX, std::vector<std::pair<size_t, size_t>>> self_collision_check_map;
            // Now, go through the links and points of the robot for collision checking
            bool any_candidate_self_collisions = false;
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future to check radius
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the transform of the current link
                const Eigen::Isometry3d link_transform = current_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    const Eigen::Vector4d environment_relative_point = link_transform * link_relative_point;
                    // Get the corresponding index
                    const std::vector<int64_t> index = LocationToExtendedGridIndex(environment_relative_point, check_resolution);
                    assert(index.size() == 3);
                    VoxelGrid::GRID_INDEX point_index(index[0], index[1], index[2]);
                    // Insert the index into the map
                    std::vector<std::pair<size_t, size_t>>& map_cell = self_collision_check_map[point_index];
                    if (map_cell.size() > 1)
                    {
                        any_candidate_self_collisions = true;
                    }
                    else if (map_cell.size() == 1)
                    {
                        const std::pair<size_t, size_t>& current = map_cell[0];
                        if (current.first != link_idx)
                        {
                            any_candidate_self_collisions = true;
                        }
                    }
                    map_cell.push_back(std::pair<size_t, size_t>(link_idx, point_idx));
                }
            }
            if (any_candidate_self_collisions == false)
            {
                return false;
            }
            // Now, we go through the map and see if any points overlap
            for (auto itr = self_collision_check_map.begin(); itr != self_collision_check_map.end(); ++itr)
            {
                const std::vector<std::pair<size_t, size_t>>& candidate_points = itr->second;
                const bool self_collision_detected = CheckPointsForSelfCollision(current_robot, candidate_points);
                if (self_collision_detected)
                {
                    return true;
                }
            }
            // If we haven't already returned, we are self-collision-free
            return false;
        }

        virtual bool CheckConfigCollision(const std::shared_ptr<BaseRobotType>& immutable_robot, const Configuration& config, const double inflation_ratio) const
        {
            std::shared_ptr<BaseRobotType> current_robot(immutable_robot->Clone());
            current_robot->SetPosition(config);
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(current_robot.get())->GetLinkGeometries();
            const double environment_collision_distance_threshold = inflation_ratio * this->environment_.GetResolution();
            const double self_collision_check_resolution = (inflation_ratio + 1.0) * this->environment_.GetResolution();
            const bool env_collision = CheckEnvironmentCollision(current_robot, robot_link_geometries, environment_collision_distance_threshold);
            const bool self_collision = CheckSelfCollisions(current_robot, robot_link_geometries, self_collision_check_resolution);
            //std::cout << self_collisions.size() << " self-colliding points to resolve" << std::endl;
            if (env_collision || self_collision)
            {
                return true;
            }
            else
            {
                return false;
            }
        }

        inline std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>> CheckCollision(const std::shared_ptr<BaseRobotType>& robot, const Configuration& previous_config, const Configuration& current_config, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const double time_interval) const
        {
            std::shared_ptr<BaseRobotType> current_robot(robot->Clone());
            std::shared_ptr<BaseRobotType> previous_robot(robot->Clone());
            // We need our own copies with a set config to use for kinematics!
            current_robot->SetPosition(current_config);
            previous_robot->SetPosition(previous_config);
            const bool env_collision = CheckEnvironmentCollision(current_robot, robot_link_geometries, contact_distance_threshold_);
            const std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d> self_collisions = CollectSelfCollisions(previous_robot, current_robot, robot_link_geometries, time_interval);
            //std::cout << self_collisions.size() << " self-colliding points to resolve" << std::endl;
            if (env_collision || (self_collisions.size() > 0))
            {
                return std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>>(true, self_collisions);
            }
            else
            {
                return std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>>(false, self_collisions);
            }
        }

        inline double EstimatePointPenetration(const Eigen::Vector4d& point) const
        {
            const std::pair<double, bool> distance_check = this->environment_sdf_.EstimateDistance(point);
            if (distance_check.second)
            {
                const double current_penetration = distance_check.first;
                if (current_penetration < this->resolution_distance_threshold_)
                {
                    return std::abs(this->resolution_distance_threshold_ - current_penetration);
                }
                else
                {
                    return 0.0;
                }
            }
            else
            {
                return std::numeric_limits<double>::infinity();
            }
        }

        inline double ComputeMaxPointPenetration(const std::shared_ptr<BaseRobotType>& current_robot, const Configuration& previous_config) const
        {
            UNUSED(previous_config);
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(current_robot.get())->GetLinkGeometries();
            // Find the point on the robot that moves the most
            double max_point_penetration = 0.0;
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future to check radius
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the *current* transform of the current link
                const Eigen::Isometry3d current_link_transform = current_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    // Get the current world position
                    const Eigen::Vector4d current_environment_position = current_link_transform * link_relative_point;
                    const double penetration = EstimatePointPenetration(current_environment_position);
                    if (penetration > max_point_penetration)
                    {
                        max_point_penetration = penetration;
                    }
                }
            }
            return max_point_penetration;
        }

        inline double EstimateMaxControlInputWorkspaceMotion(const std::shared_ptr<BaseRobotType>& start_robot, const std::shared_ptr<BaseRobotType>& end_robot) const
        {
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(start_robot.get())->GetLinkGeometries();
            // Find the point on the robot that moves the most
            double max_point_motion_squared = 0.0;
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future for radius info
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the *current* transform of the current link
                const Eigen::Isometry3d current_link_transform = start_robot->GetLinkTransform(link_name);
                // Get the *next* transform of the current link
                const Eigen::Isometry3d next_link_transform = end_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    // Get the current world position
                    const Eigen::Vector4d current_environment_position = current_link_transform * link_relative_point;
                    // Get the next world position
                    const Eigen::Vector4d next_environment_position = next_link_transform * link_relative_point;
                    // Compute the movement of the point
                    const double point_motion_squared = (next_environment_position - current_environment_position).squaredNorm();
                    if (point_motion_squared > max_point_motion_squared)
                    {
                        max_point_motion_squared = point_motion_squared;
                    }
                }
            }
            return sqrt(max_point_motion_squared);
        }

        inline double EstimateMaxControlInputWorkspaceMotion(const std::shared_ptr<BaseRobotType>& robot, const Configuration& start, const Configuration& end) const
        {
            std::shared_ptr<BaseRobotType> start_robot(robot->Clone());
            std::shared_ptr<BaseRobotType> end_robot(robot->Clone());
            start_robot->SetPosition(start);
            end_robot->SetPosition(end);
            return EstimateMaxControlInputWorkspaceMotion(start_robot, end_robot);
        }

        inline double EstimateMaxControlInputWorkspaceMotion(const std::shared_ptr<BaseRobotType>& current_robot, const Eigen::VectorXd& control_input) const
        {
            // Apply the control input
            std::shared_ptr<BaseRobotType> next_robot(current_robot->Clone());
            static_cast<DerivedRobotType*>(next_robot.get())->ApplyControlInput(control_input);
            return EstimateMaxControlInputWorkspaceMotion(current_robot, next_robot);
        }

        inline std::pair<Configuration, std::pair<bool, bool>> ResolveForwardSimulation(const std::shared_ptr<BaseRobotType>& immutable_robot, const Eigen::VectorXd& control_input, const double controller_interval, RNG& rng, const bool use_individual_jacobians, const bool allow_contacts, simple_simulator_interface::ForwardSimulationStepTrace<Configuration, ConfigAlloc>& trace, const bool enable_tracing, const uint64_t call_number, const std::function<void(const visualization_msgs::MarkerArray&)>& display_fn) const
        {
            std::shared_ptr<BaseRobotType> robot(immutable_robot->Clone());
            const Eigen::VectorXd real_control_input = control_input * controller_interval;
            if (this->debug_level_ >= 25)
            {
                const std::string msg1 = "[" + std::to_string(call_number) + "] Resolving control input: " + PrettyPrint::PrettyPrint(control_input) + "\nReal control input: " + PrettyPrint::PrettyPrint(real_control_input);
                std::cout << msg1 << std::endl;
            }
            // Get the list of link name + link points for all the links of the robot
            const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries = static_cast<DerivedRobotType*>(robot.get())->GetLinkGeometries();
            // Step along the control input
            // First, figure out how much workspace motion is actually going to result from the control input
            const double computed_step_motion = EstimateMaxControlInputWorkspaceMotion(robot, real_control_input);
            const double target_microstep_distance = this->GetResolution() * 0.125;
            const double allowed_microstep_distance = this->GetResolution() * 1.0;
            const uint32_t number_microsteps = std::max(1u, ((uint32_t)ceil(computed_step_motion / target_microstep_distance)));
            if (this->debug_level_ >= 2)
            {
                const std::string msg3 = "[" + std::to_string(call_number) + "] Resolving simulation step with computed motion: " + std::to_string(computed_step_motion) + " in " + std::to_string(number_microsteps) + " microsteps";
                std::cout << msg3 << std::endl;
            }
            const Eigen::VectorXd control_input_step = real_control_input / (double)number_microsteps;
            const double computed_microstep_motion = EstimateMaxControlInputWorkspaceMotion(robot, control_input_step);
            if (computed_microstep_motion > allowed_microstep_distance)
            {
                const std::string msg = "[" + std::to_string(call_number) + "] Computed microstep motion: " + std::to_string(computed_microstep_motion) + " > allowed " + std::to_string(allowed_microstep_distance);
                std::cerr << msg << std::endl;
                assert(false);
            }
            if (this->debug_level_ >= 25)
            {
                const std::string msg4 = "[" + std::to_string(call_number) + "] Control input step: " + PrettyPrint::PrettyPrint(control_input_step);
                std::cout << msg4 << std::endl;
            }
            bool collided = false;
            bool fallback_required = false;
            if (enable_tracing)
            {
                trace.resolver_steps.emplace_back();
                trace.resolver_steps.back().control_input = real_control_input;
                trace.resolver_steps.back().control_input_step = control_input_step;
            }
            // Iterate
            for (uint32_t micro_step = 0; micro_step < number_microsteps; micro_step++)
            {
                if (enable_tracing)
                {
                    trace.resolver_steps.back().contact_resolver_steps.emplace_back();
                }
                // Store the previous configuration of the robot
                const Configuration previous_configuration = robot->GetPosition();
                // Update the position of the robot
                static_cast<DerivedRobotType*>(robot.get())->ApplyControlInput(control_input_step, rng);
                const Configuration post_action_configuration = robot->GetPosition();
                robot->SetPosition(post_action_configuration);
                const double apply_step_max_motion = EstimateMaxControlInputWorkspaceMotion(robot, previous_configuration, post_action_configuration);
                if (this->debug_level_ >= 25)
                {
                    const std::string msg5 = "\x1b[35;1m [" + std::to_string(call_number) + "] Pre-action configuration: " + PrettyPrint::PrettyPrint(previous_configuration) + " \x1b[0m\n\x1b[33;1m [" + std::to_string(call_number) + "] Control input step: " + PrettyPrint::PrettyPrint(control_input_step) + "\n\x1b[33;1m [" + std::to_string(call_number) + "] Post-action configuration: " + PrettyPrint::PrettyPrint(post_action_configuration) + " \x1b[0m\n[" + std::to_string(call_number) + "] Max point motion (apply step) was: " + std::to_string(apply_step_max_motion);
                    std::cout << msg5 << std::endl;
                }
                std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>> collision_check = CheckCollision(robot, previous_configuration, post_action_configuration, robot_link_geometries, controller_interval);
                std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>& self_collision_map = collision_check.second;
                bool in_collision = collision_check.first;
                if (in_collision)
                {
                    collided = true;
                }
                if (enable_tracing)
                {
                    trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(post_action_configuration);
                }
                // Now, we know if a collision has happened
                if (in_collision && allow_contacts)
                {
                    Configuration active_configuration = post_action_configuration;
                    uint32_t resolver_iterations = 0;
                    double correction_step_scaling = solver_config_.resolve_correction_initial_step_size;
                    while (in_collision)
                    {
                        const auto point_jacobians_and_corrections = CollectPointCorrectionsAndJacobians(robot, previous_configuration, active_configuration, robot_link_geometries, self_collision_map, call_number, rng);

                        const Eigen::VectorXd raw_correction_step = (use_individual_jacobians) ? ComputeResolverCorrectionStepIndividualJacobians(point_jacobians_and_corrections.first, point_jacobians_and_corrections.second) : ComputeResolverCorrectionStepStackedJacobian(point_jacobians_and_corrections.first, point_jacobians_and_corrections.second);
                        const double correction_step_motion_estimate = EstimateMaxControlInputWorkspaceMotion(robot, raw_correction_step);
                        double allowed_resolve_distance = allowed_microstep_distance; //std::min(apply_step_max_motion, allowed_microstep_distance * 1.0); //was 0.25
                        if (this->debug_level_ >= 25)
                        {
                            const std::string msg7 = "[" + std::to_string(call_number) + "] Raw Cstep motion estimate: " + std::to_string(correction_step_motion_estimate) + " Allowed correction step motion: " + std::to_string(allowed_resolve_distance) + " Raw Cstep: " + PrettyPrint::PrettyPrint(raw_correction_step);
                            std::cout << msg7 << std::endl;
                        }
                        // Scale down the size of the correction step
                        // This provides additional help debugging, but performance is stable enough without it, and it reduces the number of collision checks significantly
#ifdef USE_CHECKED_RESOLVE_POINT_PENETRATION
                        const double pre_resolve_max_penetration = ComputeMaxPointPenetration(robot, previous_configuration);
                        uint32_t num_micro_ops = 0u;
                        while (num_micro_ops <= 2)
                        {
                            const double step_fraction = std::max((correction_step_motion_estimate / allowed_resolve_distance), 1.0);
                            const Eigen::VectorXd real_correction_step = (raw_correction_step / step_fraction) * std::abs(correction_step_scaling);
                            if (this->debug_level_ >= 25)
                            {
                                const std::string msg8 = "[" + std::to_string(call_number) + "] Real scaled Cstep: " + PrettyPrint::PrettyPrint(real_correction_step);
                                std::cout << msg8 << std::endl;
                            }
                            // Apply correction step
                            static_cast<DerivedRobotType*>(robot.get())->ApplyControlInput(real_correction_step);
                            const double post_resolve_max_penetration = ComputeMaxPointPenetration(robot, previous_configuration);
                            num_micro_ops++;
                            if (post_resolve_max_penetration <= pre_resolve_max_penetration)
                            {
                                break;
                            }
                            else
                            {
                                allowed_resolve_distance *= 0.75;
                                robot->SetPosition(active_configuration);
                            }
                        }
                        const Configuration post_resolve_configuration = robot->GetPosition();
                        const double post_resolve_max_penetration = ComputeMaxPointPenetration(robot, previous_configuration);
                        assert(post_resolve_max_penetration <= pre_resolve_max_penetration);
                        if (this->debug_level_ >= 25)
                        {
                            const double resolve_step_max_motion = EstimateMaxControlInputWorkspaceMotion(robot, post_action_configuration, post_resolve_configuration);
                            const double iteration_max_motion = EstimateMaxControlInputWorkspaceMotion(robot, previous_configuration, post_resolve_configuration);
                            const std::string msg9 = "\x1b[36;1m [" + std::to_string(call_number) + "] Post-resolve step configuration after " + std::to_string(num_micro_ops) + " micro-ops: " + PrettyPrint::PrettyPrint(post_resolve_configuration) + " \x1b[0m\n[" + std::to_string(call_number) + "] Pre-resolve max penetration was: " + std::to_string(pre_resolve_max_penetration) + " Post-resolve max penetration was: " + std::to_string(post_resolve_max_penetration) + " Max point motion (resolve step) was: " + std::to_string(resolve_step_max_motion) + "\n[" + std::to_string(call_number) + "] Max point motion (iteration) was: " + std::to_string(iteration_max_motion);
                            std::cout << msg9 << std::endl;
                        }
                        if (this->debug_level_ >= 26)
                        {
                            std::cout << "Press ENTER to continue..." << std::endl;
                            std::cin.get();
                        }
#else
                        const double step_fraction = std::max((correction_step_motion_estimate / allowed_resolve_distance), 1.0);
                        const Eigen::VectorXd real_correction_step = (raw_correction_step / step_fraction) * std::abs(correction_step_scaling);
                        if (this->debug_level_ >= 25)
                        {
                            const std::string msg8 = "[" + std::to_string(call_number) + "] Real scaled Cstep: " + PrettyPrint::PrettyPrint(real_correction_step);
                            std::cout << msg8 << std::endl;
                        }
                        // Apply correction step
                        static_cast<DerivedRobotType*>(robot.get())->ApplyControlInput(real_correction_step);
                        const Configuration post_resolve_configuration = robot->GetPosition();
#endif
                        active_configuration = post_resolve_configuration;
                        // Check to see if we're still in collision
                        const std::pair<bool, std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>> new_collision_check = CheckCollision(robot, previous_configuration, active_configuration, robot_link_geometries, controller_interval);
                        // Update the self-collision map
                        self_collision_map = new_collision_check.second;
                        // Update the collision check variable
                        in_collision = new_collision_check.first;
                        resolver_iterations++;
                        // Update tracing
                        if (enable_tracing)
                        {
                            trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(active_configuration);
                        }
                        if (resolver_iterations > solver_config_.max_resolver_iterations)
                        {
                            if (this->debug_level_ >= 2)
                            {
                                const std::string msg10 = "\x1b[31;1m [" + std::to_string(call_number) + "] Resolver iterations > " + std::to_string(solver_config_.max_resolver_iterations) + ", terminating microstep+resolver at configuration " + PrettyPrint::PrettyPrint(active_configuration) + " and returning previous configuration " + PrettyPrint::PrettyPrint(previous_configuration) + "\nCollision check results:\n" + PrettyPrint::PrettyPrint(new_collision_check, false, "\n") + " \x1b[0m";
                                std::cout << msg10 << std::endl;
                            }
                            if (enable_tracing)
                            {
                                trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(previous_configuration);
                            }
                            unsuccessful_resolves_.fetch_add(1);
                            if (self_collision_map.size() > 0)
                            {
                                unsuccessful_self_collision_resolves_.fetch_add(1);
                            }
                            else
                            {
                                unsuccessful_env_collision_resolves_.fetch_add(1);
                            }
                            if (this->debug_level_ >= 2)
                            {
                                const std_msgs::ColorRGBA config_color = (self_collision_map.size() > 0) ? this->MakeColor(0.5, 0.0, 0.0, 1.0) : this->MakeColor(0.0, 0.0, 0.5, 1.0);
                                const std::string active_config_markers_ns = (self_collision_map.size() > 0) ? "failed_self_collision_resolves" : "failed_env_collision_resolves";
                                visualization_msgs::MarkerArray active_config_markers = this->MakeConfigurationDisplayRep(robot, active_configuration, config_color, 1, active_config_markers_ns);
                                const int32_t active_config_markers_idx_offset = (self_collision_map.size() > 0) ? (int32_t)unsuccessful_self_collision_resolves_.load() * (int32_t)active_config_markers.markers.size() : (int32_t)unsuccessful_env_collision_resolves_.load() * (int32_t)active_config_markers.markers.size();
                                for (size_t mdx = 0; mdx < active_config_markers.markers.size(); mdx++)
                                {
                                    active_config_markers.markers[mdx].id += active_config_markers_idx_offset;
                                }
                                display_fn(active_config_markers);
                            }
                            if (this->debug_level_ >= 30)
                            {
                                assert(false);
                            }
                            if (fallback_required)
                            {
                                fallback_resolves_.fetch_add(1u);
                            }
                            return std::make_pair(previous_configuration, std::make_pair(true, true));
                        }
                        if ((resolver_iterations % solver_config_.resolve_correction_step_scaling_decay_iterations) == 0)
                        {
                            if (correction_step_scaling >= 0.0)
                            {
                                correction_step_scaling = correction_step_scaling * solver_config_.resolve_correction_step_scaling_decay_rate;
                                if (correction_step_scaling < solver_config_.resolve_correction_min_step_scaling)
                                {
                                    correction_step_scaling = -solver_config_.resolve_correction_min_step_scaling;
                                }
                            }
                            else
                            {
                                correction_step_scaling = -solver_config_.resolve_correction_min_step_scaling;
                            }
                        }
                    }
                    if (this->debug_level_ >= 25)
                    {
                        const std::string msg11 = "\x1b[31;1m [" + std::to_string(call_number) + "] Colliding microstep " + std::to_string(micro_step + 1u) + " resolved in " + std::to_string(resolver_iterations) + " iterations \x1b[0m";
                        std::cout << msg11 << std::endl;
                    }
                }
                else if (in_collision && (allow_contacts == false))
                {
                    if (this->debug_level_ >= 25)
                    {
                        const std::string msg12 = "\x1b[31;1m [" + std::to_string(call_number) + "] Colliding microstep " + std::to_string(micro_step + 1u) + " trivially resolved in 1 iteration (allow_contacts==false) \x1b[0m";
                        std::cout << msg12 << std::endl;
                    }
                    if (enable_tracing)
                    {
                        trace.resolver_steps.back().contact_resolver_steps.back().contact_resolution_steps.push_back(previous_configuration);
                    }
                    successful_resolves_.fetch_add(1);
                    if (fallback_required)
                    {
                        fallback_resolves_.fetch_add(1u);
                    }
                    return std::make_pair(previous_configuration, std::make_pair(true, false));
                }
                else
                {
                    if (this->debug_level_ >= 25)
                    {
                        const std::string msg13= "\x1b[31;1m [" + std::to_string(call_number) + "] Uncolliding microstep " + std::to_string(micro_step + 1u) + " trivially resolved in 1 iteration \x1b[0m";
                        std::cout << msg13 << std::endl;
                    }
                    continue;
                }
            }
            if (this->debug_level_ >= 2)
            {
                const std::string msg14 = "\x1b[32;1m [" + std::to_string(call_number) + "] Resolved action, post-action resolution configuration: " + PrettyPrint::PrettyPrint(robot->GetPosition()) + " \x1b[0m";
                std::cout << msg14 << std::endl;
            }
            successful_resolves_.fetch_add(1);
            if (fallback_required)
            {
                fallback_resolves_.fetch_add(1u);
            }
            if (collided)
            {
                collision_resolves_.fetch_add(1);
            }
            else
            {
                free_resolves_.fetch_add(1);
            }
            return std::make_pair(robot->GetPosition(), std::make_pair(collided, false));
        }

        inline std::pair<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>, Eigen::Matrix<double, Eigen::Dynamic, 1>> CollectPointCorrectionsAndJacobians(const std::shared_ptr<BaseRobotType>& robot, const Configuration& previous_config, const Configuration& current_config, const std::vector<std::pair<std::string, simple_robot_models::PointSphereGeometry>>& robot_link_geometries, const std::unordered_map<std::pair<size_t, size_t>, Eigen::Vector3d>& self_collision_map, const uint64_t call_number, RNG& rng) const
        {
            UNUSED(rng);
            std::shared_ptr<BaseRobotType> current_robot(robot->Clone());
            std::shared_ptr<BaseRobotType> previous_robot(robot->Clone());
            // We need our own copy with a set config to use for kinematics!
            current_robot->SetPosition(current_config);
            previous_robot->SetPosition(previous_config);
            // In case a collision has occured, we need to compute a "collision gradient" that will push the robot out of collision
            // The "collision gradient" is of the form qgradient = J(q)+ * xgradient
            // Make space for the xgradient and Jacobian
            Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> robot_jacobians;
            Eigen::Matrix<double, Eigen::Dynamic, 1> point_corrections;
            // Now, go through the links and points of the robot to build up the xgradient and Jacobian
            for (size_t link_idx = 0; link_idx < robot_link_geometries.size(); link_idx++)
            {
                // Grab the link name and points
                const std::string& link_name = robot_link_geometries[link_idx].first;
                const simple_robot_models::PointSphereGeometry& link_geometry = robot_link_geometries[link_idx].second;
                // Use this in the future for radius info
                //const simple_robot_models::PointSphereGeometry::MODEL_GEOMETRY_TYPE& link_geometry_type = link_geometry.GeometryType();
                const EigenHelpers::VectorVector4d& link_points = *(link_geometry.Geometry());
                // Get the transform of the current link
                const Eigen::Isometry3d previous_link_transform = previous_robot->GetLinkTransform(link_name);
                const Eigen::Isometry3d current_link_transform = current_robot->GetLinkTransform(link_name);
                // Now, go through the points of the link
                for (size_t point_idx = 0; point_idx < link_points.size(); point_idx++)
                {
                    // Check if we have a self-collision correction
                    std::pair<bool, Eigen::Vector3d> self_collision_correction(false, Eigen::Vector3d(0.0, 0.0, 0.0));
                    auto self_collision_check = self_collision_map.find(std::pair<size_t, size_t>(link_idx, point_idx));
                    if (self_collision_check != self_collision_map.end())
                    {
                        self_collision_correction.first = true;
                        self_collision_correction.second = self_collision_check->second;
                    }
                    // Check against the environment
                    std::pair<bool, Eigen::Vector3d> env_collision_correction(false, Eigen::Vector3d(0.0, 0.0, 0.0));
                    const Eigen::Vector4d& link_relative_point = link_points[point_idx];
                    // Get the Jacobian for the current point
                    const Eigen::Matrix<double, 3, Eigen::Dynamic> point_jacobian = current_robot->ComputeLinkPointTranslationJacobian(link_name, link_relative_point);
                    // Transform the link point into the environment frame
                    const Eigen::Vector4d previous_point_location = previous_link_transform * link_relative_point;
                    const Eigen::Vector4d current_point_location = current_link_transform * link_relative_point;
                    // We only work with points in the SDF
                    const std::pair<double, bool> current_sdf_check = this->environment_sdf_.EstimateDistance4d(current_point_location);
                    if (current_sdf_check.second == false)
                    {
                        const std::string msg = "[" + std::to_string(call_number) + "] (Current) Point out of bounds: " + PrettyPrint::PrettyPrint(current_point_location);
                        arc_helpers::ConditionalPrint(msg, 0, this->debug_level_);
                        if (this->debug_level_ >= 5)
                        {
                            assert(false);
                        }
                    }
                    // We only work with points in collision
                    if (current_sdf_check.first < this->resolution_distance_threshold_ && current_sdf_check.second)
                    {
                        // In the future, we might want to consider reversing the point motion if the robot started in contact and we're trying to leave collision
                        // We query the surface normal map for the gradient to move out of contact using the particle motion
                        const Eigen::Vector4d point_motion = current_point_location - previous_point_location;
                        const Eigen::Vector4d normed_point_motion = EigenHelpers::SafeNormal(point_motion);
                        // Query the surface normal map
                        const std::pair<Eigen::Vector3d, bool> surface_normal_query = this->surface_normals_grid_.LookupSurfaceNormal(current_point_location, normed_point_motion);
                        assert(surface_normal_query.second);
                        const Eigen::Vector3d& raw_gradient = surface_normal_query.first;
                        const Eigen::Vector3d normed_point_gradient = EigenHelpers::SafeNormal(raw_gradient);
                        // Compute the penetration weight
                        const double sdf_penetration_distance = std::abs(this->resolution_distance_threshold_ - current_sdf_check.first);
                        // Compute the correction vector
                        const Eigen::Vector3d correction_vector = normed_point_gradient * sdf_penetration_distance;
                        env_collision_correction.first = true;
                        env_collision_correction.second = correction_vector;
                    }
                    // We only add a correction for the point if necessary
                    if (self_collision_correction.first || env_collision_correction.first)
                    {
                        // Append the new point jacobian to the matrix of jacobians
                        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> extended_robot_jacobians;
                        extended_robot_jacobians.resize(robot_jacobians.rows() + 3, point_jacobian.cols());
                        if (robot_jacobians.cols() > 0)
                        {
                            extended_robot_jacobians << robot_jacobians,point_jacobian;
                        }
                        else
                        {
                            extended_robot_jacobians << point_jacobian;
                        }
                        robot_jacobians = extended_robot_jacobians;
                        // Assemble the workspace correction vector
                        Eigen::Vector3d point_correction(0.0, 0.0, 0.0);
                        if (self_collision_correction.first)
                        {
                            if (this->debug_level_ >= 25)
                            {
                                std::cout << "Self-collision correction: " << PrettyPrint::PrettyPrint(self_collision_correction.second) << std::endl;
                            }
                            point_correction = point_correction + self_collision_correction.second;
                        }
                        if (env_collision_correction.first)
                        {
                            if (this->debug_level_ >= 25)
                            {
                                std::cout << "Env-collision correction: " << PrettyPrint::PrettyPrint(env_collision_correction.second) << std::endl;
                            }
                            point_correction = point_correction + env_collision_correction.second;
                        }
                        // Append the new workspace correction vector to the matrix of correction vectors
                        Eigen::Matrix<double, Eigen::Dynamic, 1> extended_point_corrections;
                        extended_point_corrections.resize(point_corrections.rows() + 3, Eigen::NoChange);
                        extended_point_corrections << point_corrections,point_correction;
                        point_corrections = extended_point_corrections;
                        if (this->debug_level_ >= 35)
                        {
                            std::cout << "Point jacobian:\n" << point_jacobian << std::endl;
                            std::cout << "Point correction: " << PrettyPrint::PrettyPrint(point_correction) << std::endl;
                        }
                    }
                }
            }
            return std::make_pair(robot_jacobians, point_corrections);
        }

        inline Eigen::VectorXd ComputeResolverCorrectionStepJacobiansTranspose(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_jacobians, const Eigen::Matrix<double, Eigen::Dynamic, 1>& point_corrections) const
        {
            // In case a collision has occured, we need to compute a "collision gradient" that will push the robot out of collision
            Eigen::VectorXd raw_correction_step;
            // Go through the collected jacobians and corrections
            for (ssize_t idx = 0; idx < robot_jacobians.rows(); idx += 3)
            {
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& point_jacobian = robot_jacobians.block(idx, 0, 3, robot_jacobians.cols());
                const Eigen::Vector3d& point_correction = point_corrections.block<3, 1>(idx, 0);
                // Compute the correction step T = J(q,p).t * F
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_correction = point_jacobian.transpose() * point_correction;
                // Extract the c-space correction
                const Eigen::VectorXd point_correction_step = raw_correction.col(0);
                if (raw_correction_step.size() == 0)
                {
                    raw_correction_step = point_correction_step;
                }
                else
                {
                    raw_correction_step = raw_correction_step + point_correction_step;
                }
            }
            return raw_correction_step;
        }

        inline Eigen::VectorXd ComputeResolverCorrectionStepIndividualJacobians(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_jacobians, const Eigen::Matrix<double, Eigen::Dynamic, 1>& point_corrections) const
        {
            // In case a collision has occured, we need to compute a "collision gradient" that will push the robot out of collision
            Eigen::VectorXd raw_correction_step;
            // Go through the collected jacobians and corrections
            for (ssize_t idx = 0; idx < robot_jacobians.rows(); idx += 3)
            {
                const Eigen::Matrix<double, 3, Eigen::Dynamic>& point_jacobian = robot_jacobians.block(idx, 0, 3, robot_jacobians.cols());
                const Eigen::Vector3d& point_correction = point_corrections.block<3, 1>(idx, 0);
                const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_correction = point_jacobian.colPivHouseholderQr().solve(point_correction);
                // Extract the c-space correction
                const Eigen::VectorXd point_correction_step = raw_correction.col(0);
                if (raw_correction_step.size() == 0)
                {
                    raw_correction_step = point_correction_step;
                }
                else
                {
                    raw_correction_step = raw_correction_step + point_correction_step;
                }
            }
            return raw_correction_step;
        }

        inline Eigen::VectorXd ComputeResolverCorrectionStepStackedJacobian(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& robot_jacobians, const Eigen::Matrix<double, Eigen::Dynamic, 1>& point_corrections) const
        {
            // Compute the correction step
            // We could use the naive Pinv(J) * pdot, but instead we solve the Ax = b (Jqdot = pdot) problem directly using one of the solvers in Eigen
            const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> raw_correction = robot_jacobians.colPivHouseholderQr().solve(point_corrections);
            // Extract the c-space correction
            const Eigen::VectorXd raw_correction_step = raw_correction.col(0);
            return raw_correction_step;
        }
    };
}

#endif // SIMPLE_PARTICLE_CONTACT_SIMULATOR_HPP
