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
#include <arc_utilities/eigen_helpers.hpp>
#include <arc_utilities/eigen_helpers_conversions.hpp>
#include <arc_utilities/voxel_grid.hpp>
#include <sdf_tools/tagged_object_collision_map.hpp>
#include <sdf_tools/sdf.hpp>
#include <uncertainty_planning_core/simple_simulator_interface.hpp>
#include <fast_kinematic_simulator/simulator_environment_builder.hpp>

/* Discretize a cuboid obstacle to resolution-sized cells */
std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> simulator_environment_builder::DiscretizeObstacle(const OBSTACLE_CONFIG& obstacle, const double resolution)
{
    const double effective_resolution = resolution * 0.5;
    std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> cells;
    // Make the cell for the object
    sdf_tools::TAGGED_OBJECT_COLLISION_CELL object_cell(1.0, obstacle.object_id);
    // Generate all cells for the object
    int32_t x_cells = (int32_t)(obstacle.extents.x() * 2.0 * (1.0 / effective_resolution));
    int32_t y_cells = (int32_t)(obstacle.extents.y() * 2.0 * (1.0 / effective_resolution));
    int32_t z_cells = (int32_t)(obstacle.extents.z() * 2.0 * (1.0 / effective_resolution));
    for (int32_t xidx = 0; xidx < x_cells; xidx++)
    {
        for (int32_t yidx = 0; yidx < y_cells; yidx++)
        {
            for (int32_t zidx = 0; zidx < z_cells; zidx++)
            {
                double x_location = -(obstacle.extents.x() - (resolution * 0.5)) + (effective_resolution * xidx);
                double y_location = -(obstacle.extents.y() - (resolution * 0.5)) + (effective_resolution * yidx);
                double z_location = -(obstacle.extents.z() - (resolution * 0.5)) + (effective_resolution * zidx);
                Eigen::Vector3d cell_location(x_location, y_location, z_location);
                cells.push_back(std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>(cell_location, object_cell));
            }
        }
    }
    return cells;
}

/* Build a new environment from the provided obstacles */
sdf_tools::TaggedObjectCollisionMapGrid simulator_environment_builder::BuildEnvironment(const std::vector<OBSTACLE_CONFIG>& obstacles, const double resolution)
{
    if (obstacles.empty())
    {
        std::cerr << "No obstacles provided, generating the default environment" << std::endl;
        double grid_x_size = 10.0;
        double grid_y_size = 10.0;
        double grid_z_size = 10.0;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(0.0, 0.0, 0.0);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Isometry3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        return grid;
    }
    else
    {
        std::cout << "Rebuilding the environment with " << obstacles.size() << " obstacles" << std::endl;
        // We need to loop through the obstacles, discretize each obstacle, and then find the size of the grid we need to store them
        bool xyz_bounds_initialized = false;
        double x_min = 0.0;
        double y_min = 0.0;
        double z_min = 0.0;
        double x_max = 0.0;
        double y_max = 0.0;
        double z_max = 0.0;
        std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> all_obstacle_cells;
        for (size_t idx = 0; idx < obstacles.size(); idx++)
        {
            const OBSTACLE_CONFIG& obstacle = obstacles[idx];
            std::vector<std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>> obstacle_cells = DiscretizeObstacle(obstacle, resolution);
            for (size_t cidx = 0; cidx < obstacle_cells.size(); cidx++)
            {
                const Eigen::Vector3d& relative_location = obstacle_cells[cidx].first;
                Eigen::Vector3d real_location = obstacle.pose * relative_location;
                all_obstacle_cells.push_back(std::pair<Eigen::Vector3d, sdf_tools::TAGGED_OBJECT_COLLISION_CELL>(real_location, obstacle_cells[cidx].second));
                // Check against the min/max extents
                if (xyz_bounds_initialized)
                {
                    if (real_location.x() < x_min)
                    {
                        x_min = real_location.x();
                    }
                    else if (real_location.x() > x_max)
                    {
                        x_max = real_location.x();
                    }
                    if (real_location.y() < y_min)
                    {
                        y_min = real_location.y();
                    }
                    else if (real_location.y() > y_max)
                    {
                        y_max = real_location.y();
                    }
                    if (real_location.z() < z_min)
                    {
                        z_min = real_location.z();
                    }
                    else if (real_location.z() > z_max)
                    {
                        z_max = real_location.z();
                    }
                }
                // If we haven't initialized the bounds yet, set them to the current position
                else
                {
                    x_min = real_location.x();
                    x_max = real_location.x();
                    y_min = real_location.y();
                    y_max = real_location.y();
                    z_min = real_location.z();
                    z_max = real_location.z();
                    xyz_bounds_initialized = true;
                }
            }
        }
        // Now that we've done that, we fill in the grid to store them
        // Key the center of the first cell off the the minimum value
        x_min -= (resolution * 0.5);
        y_min -= (resolution * 0.5);
        z_min -= (resolution * 0.5);
        // Add a 1-cell buffer to all sides
        x_min -= (resolution * 3.0);
        y_min -= (resolution * 3.0);
        z_min -= (resolution * 3.0);
        x_max += (resolution * 3.0);
        y_max += (resolution * 3.0);
        z_max += (resolution * 3.0);
        double grid_x_size = x_max - x_min;
        double grid_y_size = y_max - y_min;
        double grid_z_size = z_max - z_min;
        // The grid origin is the minimum point, with identity rotation
        Eigen::Translation3d grid_origin_translation(x_min, y_min, z_min);
        Eigen::Quaterniond grid_origin_rotation = Eigen::Quaterniond::Identity();
        Eigen::Isometry3d grid_origin_transform = grid_origin_translation * grid_origin_rotation;
        // Make the grid
        sdf_tools::TAGGED_OBJECT_COLLISION_CELL default_cell;
        sdf_tools::TaggedObjectCollisionMapGrid grid(grid_origin_transform, "uncertainty_planning_simulator", resolution, grid_x_size, grid_y_size, grid_z_size, default_cell);
        // Fill it in
        for (size_t idx = 0; idx < all_obstacle_cells.size(); idx++)
        {
            const Eigen::Vector3d& location = all_obstacle_cells[idx].first;
            const sdf_tools::TAGGED_OBJECT_COLLISION_CELL& cell = all_obstacle_cells[idx].second;
            grid.SetValue(location.x(), location.y(), location.z(), cell);
        }
        // Set the environment
        return grid;
    }
}

void simulator_environment_builder::UpdateSurfaceNormalGridCell(const std::vector<RawCellSurfaceNormal>& raw_surface_normals, const Eigen::Isometry3d& transform, const Eigen::Vector3d& cell_location, const sdf_tools::SignedDistanceField& environment_sdf, simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid)
{
    const Eigen::Vector3d world_location = transform * cell_location;
    // Let's check the penetration distance. We only want to update cells that are *actually* on the surface
    const float distance = environment_sdf.GetImmutable3d(world_location).Value();
    // If we're within one cell of the surface, we update
    if (distance > -(environment_sdf.GetResolution() * 1.5))
    {
        // First, we clear any stored surface normals
        surface_normals_grid.ClearStoredSurfaceNormals(world_location);
        for (size_t idx = 0; idx < raw_surface_normals.size(); idx++)
        {
            const RawCellSurfaceNormal& current_surface_normal = raw_surface_normals[idx];
            const Eigen::Vector3d& raw_surface_normal = current_surface_normal.normal;
            const Eigen::Vector3d& raw_entry_direction = current_surface_normal.entry_direction;
            const Eigen::Vector3d real_surface_normal = (Eigen::Vector3d)(transform.rotation() * raw_surface_normal);
            const Eigen::Vector3d real_entry_direction = (Eigen::Vector3d)(transform.rotation() * raw_entry_direction);
            surface_normals_grid.InsertSurfaceNormal(world_location, real_surface_normal, real_entry_direction);
        }
    }
    else
    {
        // Do nothing otherwise
        ;
    }
}

void simulator_environment_builder::AdjustSurfaceNormalGridForAllFlatSurfaces(const sdf_tools::SignedDistanceField& environment_sdf, simple_particle_contact_simulator::SurfaceNormalGrid& surface_normals_grid)
{
    for (int64_t x_idx = 0; x_idx < environment_sdf.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment_sdf.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment_sdf.GetNumZCells(); z_idx++)
            {
                const float distance = environment_sdf.GetImmutable(x_idx, y_idx, z_idx).Value();
                if (distance < 0.0)
                {
                    const float xm1_distance = environment_sdf.GetImmutable(x_idx - 1, y_idx, z_idx).Value();
                    const float xp1_distance = environment_sdf.GetImmutable(x_idx + 1, y_idx, z_idx).Value();
                    const float ym1_distance = environment_sdf.GetImmutable(x_idx, y_idx - 1, z_idx).Value();
                    const float yp1_distance = environment_sdf.GetImmutable(x_idx, y_idx + 1, z_idx).Value();
                    const float zm1_distance = environment_sdf.GetImmutable(x_idx, y_idx, z_idx - 1).Value();
                    const float zp1_distance = environment_sdf.GetImmutable(x_idx, y_idx, z_idx + 1).Value();
                    const bool xm1_edge = (xm1_distance > 0.0);
                    const bool xp1_edge = (xp1_distance > 0.0);
                    const bool ym1_edge = (ym1_distance > 0.0);
                    const bool yp1_edge = (yp1_distance > 0.0);
                    const bool zm1_edge = (zm1_distance > 0.0);
                    const bool zp1_edge = (zp1_distance > 0.0);
                    if (xm1_edge || xp1_edge || ym1_edge || yp1_edge || zm1_edge || zp1_edge)
                    {
                        surface_normals_grid.ClearStoredSurfaceNormals(x_idx, y_idx, z_idx);
                        if (xm1_edge)
                        {
                            const Eigen::Vector3d normal(-1.0, 0.0, 0.0);
                            const Eigen::Vector3d entry(1.0, 0.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (xp1_edge)
                        {
                            const Eigen::Vector3d normal(1.0, 0.0, 0.0);
                            const Eigen::Vector3d entry(-1.0, 0.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (ym1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, -1.0, 0.0);
                            const Eigen::Vector3d entry(0.0, 1.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (yp1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, 1.0, 0.0);
                            const Eigen::Vector3d entry(0.0, -1.0, 0.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (zm1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, 0.0, -1.0);
                            const Eigen::Vector3d entry(0.0, 0.0, 1.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                        if (zp1_edge)
                        {
                            const Eigen::Vector3d normal(0.0, 0.0, 1.0);
                            const Eigen::Vector3d entry(0.0, 0.0, -1.0);
                            surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, normal, entry);
                        }
                    }
                }
            }
        }
    }
}

simple_particle_contact_simulator::SurfaceNormalGrid simulator_environment_builder::BuildSurfaceNormalsGrid(const std::vector<OBSTACLE_CONFIG>& obstacles, const sdf_tools::SignedDistanceField& environment_sdf)
{
    // Make the grid
    simple_particle_contact_simulator::SurfaceNormalGrid surface_normals_grid(environment_sdf.GetOriginTransform(), environment_sdf.GetResolution(), environment_sdf.GetXSize(), environment_sdf.GetYSize(), environment_sdf.GetZSize());
    // The naive start is to fill the surface normals grid with the gradient values from the SDF
    for (int64_t x_idx = 0; x_idx < environment_sdf.GetNumXCells(); x_idx++)
    {
        for (int64_t y_idx = 0; y_idx < environment_sdf.GetNumYCells(); y_idx++)
        {
            for (int64_t z_idx = 0; z_idx < environment_sdf.GetNumZCells(); z_idx++)
            {
                const float distance = environment_sdf.GetImmutable(x_idx, y_idx, z_idx).Value();
                if (distance < 0.0)
                {
                    const Eigen::Vector3d gradient = EigenHelpers::StdVectorDoubleToEigenVector3d(environment_sdf.GetGradient(x_idx, y_idx, z_idx, true));
                    surface_normals_grid.InsertSurfaceNormal(x_idx, y_idx, z_idx, gradient, Eigen::Vector3d(0.0, 0.0, 0.0));
                }
            }
        }
    }
    // Now, as a second pass, we go through the objects and compute the true surface normal(s) for every object
    for (size_t idx = 0; idx < obstacles.size(); idx++)
    {
        const OBSTACLE_CONFIG& current_obstacle = obstacles[idx];
        const double effective_resolution = environment_sdf.GetResolution() * 0.5;
        // Generate all cells for the object
        int32_t x_cells = (int32_t)(current_obstacle.extents.x() * 2.0 * (1.0 / effective_resolution));
        int32_t y_cells = (int32_t)(current_obstacle.extents.y() * 2.0 * (1.0 / effective_resolution));
        int32_t z_cells = (int32_t)(current_obstacle.extents.z() * 2.0 * (1.0 / effective_resolution));
        for (int32_t xidx = 0; xidx < x_cells; xidx++)
        {
            for (int32_t yidx = 0; yidx < y_cells; yidx++)
            {
                for (int32_t zidx = 0; zidx < z_cells; zidx++)
                {
                    // If we're on the edge of the obstacle
                    if ((xidx == 0) || (yidx == 0) || (zidx == 0) || (xidx == (x_cells - 1)) || (yidx == (y_cells - 1)) || (zidx == (z_cells - 1)))
                    {
                        double x_location = -(current_obstacle.extents.x() - effective_resolution) + (effective_resolution * xidx);
                        double y_location = -(current_obstacle.extents.y() - effective_resolution) + (effective_resolution * yidx);
                        double z_location = -(current_obstacle.extents.z() - effective_resolution) + (effective_resolution * zidx);
                        const Eigen::Vector3d local_cell_location(x_location, y_location, z_location);
                        // Go through all 26 cases
                        // Start with the 8 corners
                        if ((xidx == 0) && (yidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == (y_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == (y_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == (y_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == (y_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal3(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2, normal3}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        // Next, let's cover the 12 edges
                        else if ((xidx == 0) && (yidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (yidx == (y_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (yidx == (y_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((xidx == (x_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == 0) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == 0) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == (y_cells - 1)) && (zidx == 0))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if ((yidx == (y_cells - 1)) && (zidx == (z_cells - 1)))
                        {
                            const RawCellSurfaceNormal normal1(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            const RawCellSurfaceNormal normal2(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal1, normal2}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        // Finally, let's cover the 6 faces
                        else if (xidx == 0)
                        {
                            const RawCellSurfaceNormal normal(-Eigen::Vector3d::UnitX(), Eigen::Vector3d::UnitX());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (xidx == (x_cells - 1))
                        {
                            const RawCellSurfaceNormal normal(Eigen::Vector3d::UnitX(), -Eigen::Vector3d::UnitX());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (yidx == 0)
                        {
                            const RawCellSurfaceNormal normal(-Eigen::Vector3d::UnitY(), Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (yidx == (y_cells - 1))
                        {
                            const RawCellSurfaceNormal normal(Eigen::Vector3d::UnitY(), -Eigen::Vector3d::UnitY());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (zidx == 0)
                        {
                            const RawCellSurfaceNormal normal(-Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                        else if (zidx == (z_cells - 1))
                        {
                            const RawCellSurfaceNormal normal(Eigen::Vector3d::UnitZ(), -Eigen::Vector3d::UnitZ());
                            UpdateSurfaceNormalGridCell(std::vector<RawCellSurfaceNormal>{normal}, current_obstacle.pose, local_cell_location, environment_sdf, surface_normals_grid);
                        }
                    }
                }
            }
        }
    }
    return surface_normals_grid;
}

simulator_environment_builder::EnvironmentComponents simulator_environment_builder::BuildCompleteEnvironment(const std::vector<OBSTACLE_CONFIG>& obstacles, const double resolution)
{
    const sdf_tools::TaggedObjectCollisionMapGrid environment = BuildEnvironment(obstacles, resolution);
    const sdf_tools::SignedDistanceField environment_sdf = environment.ExtractSignedDistanceField(std::numeric_limits<float>::infinity(), std::vector<uint32_t>(), true, false).first;
    const simple_particle_contact_simulator::SurfaceNormalGrid surface_normal_grid = BuildSurfaceNormalsGrid(obstacles, environment_sdf);
    return simulator_environment_builder::EnvironmentComponents(environment, environment_sdf, surface_normal_grid);
}
