#pragma once

/// The free functions of the `mb_assembly` module.
///
/// The assembly layer turns an `AxleInput` into a solver-ready `Model`:
/// `build_model`, the eleven vehicle row-registration readers it calls, and the
/// generic element block reader.  The declarations are grouped by the module
/// that defines them, not by the module that calls them, so the layering the
/// project checks with `check_module_layering.py` is also the layering of these
/// headers.

#include "mb_config/prelude.hpp"
#include "mb_numeric/vector.hpp"
#include "mb_model/types.hpp"
#include "mb_input/types.hpp"

namespace axle_kernel {
// Generic element surface (K7): reads element blocks into a model.  Defined by
// the assembly layer's `element_reader.cpp`; the build path and the vehicle
// registration path both call it.
bool read_element_blocks(
    const ElementBlock* elements,
    std::size_t element_count,
    const ElementCurveReference* element_curves,
    std::size_t topology_extension_count,
    const TopologyExtensionBlock* topology_extensions,
    Model& model,
    std::string& error
);

Model build_model( const AxleInput& in, std::string& error, const double* axis_a_secondary = nullptr, const double* axis_b_secondary = nullptr, const double* convel_angle_target = nullptr, std::size_t coordinate_coupler_count = 0, const int* coordinate_coupler_joint_a = nullptr, const int* coordinate_coupler_coordinate_a = nullptr, const double* coordinate_coupler_scale_a = nullptr, const int* coordinate_coupler_joint_b = nullptr, const int* coordinate_coupler_coordinate_b = nullptr, const double* coordinate_coupler_scale_b = nullptr );

bool add_vehicle_steering_actuators( const VehicleInput& input, Model& model, std::string& error );

bool add_driven_coordinates( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_aerodynamic_drags( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_road_profile( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_static_rotation_gauges( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_tire_frames( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_tire_models( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_drive_torque_mappings( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_spring_curves( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_bushing_curves( const VehicleInput& input, Model& model, std::string& error );

bool add_vehicle_bushing_rotation_coordinates( const VehicleInput& input, Model& model, std::string& error );
} // namespace axle_kernel
