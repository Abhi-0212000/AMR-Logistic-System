# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
import rclpy.parameter
from generate_parameter_library_py.python_validators import ParameterValidators


class trajectory_planner:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        class __MapData:
            map_path = "/ros2_ws/src/laneletMaps/HS-SchmalkaldenPart1.osm"

            class __GpsOrigin:
                latitude = 50.7153508
                longitude = 10.4680332
                altitude = 0.0

            gps_origin = __GpsOrigin()

        map_data = __MapData()

        class __WindowManager:
            planning_time = 1.0
            buffer_time = 0.5
            lookahead_points = 4

        window_manager = __WindowManager()

        class __CenterlineProcessor:
            interpolation_method = "linear"
            target_spacing = 2.0
            spacing_tolerance = 0.1
            bezier_window_size = 6
            bezier_overlap = 2

        centerline_processor = __CenterlineProcessor()

        class __TrajectoryOptimization:
            trajectory_point_count = 150
            base_tangent_factor = 0.5
            optimization_time_limit = 0.8
            arc_length_calculation_method = "linear"

            class __CollisionChecking:
                safety_margin = 0.2
                collision_check_interval = 5

            collision_checking = __CollisionChecking()

            class __Rprop:
                initial_step_size = 0.3
                minimum_step_size = 0.0001
                maximum_step_size = 50.0
                increase_factor = 1.2
                decrease_factor = 0.5

            rprop = __Rprop()

        trajectory_optimization = __TrajectoryOptimization()

        class __DistanceMap:
            resolution = 0.2
            window_size = 3.0
            use_sobel = True
            sobel_threshold = 1e-06

        distance_map = __DistanceMap()

        class __RobotConstraints:
            max_velocity = 1.5
            max_acceleration = 1.0
            max_deceleration = 2.0
            max_jerk = 1.0
            max_lateral_accel = 0.5
            wheel_base = 0.5
            min_turning_radius = 1.0

        robot_constraints = __RobotConstraints()

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = trajectory_planner.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("trajectory_planner." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def unpack_parameter_dict(self, namespace: str, parameter_dict: dict):
            """
            Flatten a parameter dictionary recursively.

            :param namespace: The namespace to prepend to the parameter names.
            :param parameter_dict: A dictionary of parameters keyed by the parameter names
            :return: A list of rclpy Parameter objects
            """
            parameters = []
            for param_name, param_value in parameter_dict.items():
                full_param_name = namespace + param_name
                # Unroll nested parameters
                if isinstance(param_value, dict):
                    nested_params = self.unpack_parameter_dict(
                        namespace=full_param_name + rclpy.parameter.PARAMETER_SEPARATOR_STRING,
                        parameter_dict=param_value,
                    )
                    parameters.extend(nested_params)
                else:
                    parameters.append(
                        rclpy.parameter.Parameter(full_param_name, value=param_value)
                    )
            return parameters

        def set_params_from_dict(self, param_dict):
            params_to_set = self.unpack_parameter_dict("", param_dict)
            self.update(params_to_set)

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "map_data.map_path":
                    updated_params.map_data.map_path = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "map_data.gps_origin.latitude":
                    updated_params.map_data.gps_origin.latitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "map_data.gps_origin.longitude":
                    updated_params.map_data.gps_origin.longitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "map_data.gps_origin.altitude":
                    updated_params.map_data.gps_origin.altitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "window_manager.planning_time":
                    validation_result = ParameterValidators.bounds(param, 0.2, 5.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.window_manager.planning_time = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "window_manager.buffer_time":
                    validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.window_manager.buffer_time = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "window_manager.lookahead_points":
                    validation_result = ParameterValidators.bounds(param, 2, 20)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.window_manager.lookahead_points = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "centerline_processor.interpolation_method":
                    validation_result = ParameterValidators.one_of(param, ["linear", "bezier"])
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.centerline_processor.interpolation_method = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "centerline_processor.target_spacing":
                    validation_result = ParameterValidators.bounds(param, 0.5, 5.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.centerline_processor.target_spacing = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "centerline_processor.spacing_tolerance":
                    validation_result = ParameterValidators.bounds(param, 0.01, 0.5)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.centerline_processor.spacing_tolerance = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "centerline_processor.bezier_window_size":
                    validation_result = ParameterValidators.bounds(param, 3, 20)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.centerline_processor.bezier_window_size = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "centerline_processor.bezier_overlap":
                    validation_result = ParameterValidators.bounds(param, 1, 10)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.centerline_processor.bezier_overlap = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.trajectory_point_count":
                    validation_result = ParameterValidators.bounds(param, 50, 500)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.trajectory_point_count = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.base_tangent_factor":
                    validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.base_tangent_factor = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.optimization_time_limit":
                    validation_result = ParameterValidators.bounds(param, 0.1, 5.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.optimization_time_limit = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_ + "trajectory_optimization.arc_length_calculation_method"
                ):
                    validation_result = ParameterValidators.one_of(param, ["linear", "simpson"])
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.arc_length_calculation_method = (
                        param.value
                    )
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_ + "trajectory_optimization.collision_checking.safety_margin"
                ):
                    validation_result = ParameterValidators.bounds(param, 0.05, 1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.collision_checking.safety_margin = (
                        param.value
                    )
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if (
                    param.name
                    == self.prefix_
                    + "trajectory_optimization.collision_checking.collision_check_interval"
                ):
                    validation_result = ParameterValidators.bounds(param, 1, 10)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.collision_checking.collision_check_interval = (
                        param.value
                    )
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.rprop.initial_step_size":
                    validation_result = ParameterValidators.bounds(param, 0.01, 1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.rprop.initial_step_size = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.rprop.minimum_step_size":
                    validation_result = ParameterValidators.bounds(param, 1e-06, 0.01)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.rprop.minimum_step_size = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.rprop.maximum_step_size":
                    validation_result = ParameterValidators.bounds(param, 1.0, 100.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.rprop.maximum_step_size = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.rprop.increase_factor":
                    validation_result = ParameterValidators.bounds(param, 1.0, 2.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.rprop.increase_factor = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "trajectory_optimization.rprop.decrease_factor":
                    validation_result = ParameterValidators.bounds(param, 0.1, 0.9)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.trajectory_optimization.rprop.decrease_factor = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "distance_map.resolution":
                    validation_result = ParameterValidators.bounds(param, 0.05, 0.5)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.distance_map.resolution = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "distance_map.window_size":
                    validation_result = ParameterValidators.bounds(param, 1.0, 10.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.distance_map.window_size = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "distance_map.use_sobel":
                    updated_params.distance_map.use_sobel = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "distance_map.sobel_threshold":
                    validation_result = ParameterValidators.bounds(param, 1e-09, 0.001)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.distance_map.sobel_threshold = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.max_velocity":
                    validation_result = ParameterValidators.bounds(param, 0.1, 3.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.max_velocity = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.max_acceleration":
                    validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.max_acceleration = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.max_deceleration":
                    validation_result = ParameterValidators.bounds(param, 0.1, 3.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.max_deceleration = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.max_jerk":
                    validation_result = ParameterValidators.bounds(param, 0.1, 5.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.max_jerk = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.max_lateral_accel":
                    validation_result = ParameterValidators.bounds(param, 0.1, 1.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.max_lateral_accel = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.wheel_base":
                    validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.wheel_base = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "robot_constraints.min_turning_radius":
                    validation_result = ParameterValidators.bounds(param, 0.5, 5.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.robot_constraints.min_turning_radius = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "map_data.map_path"):
                descriptor = ParameterDescriptor(
                    description="Path to the lanelet2 map file (.osm format)", read_only=True
                )
                parameter = updated_params.map_data.map_path
                self.node_.declare_parameter(
                    self.prefix_ + "map_data.map_path", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "map_data.gps_origin.latitude"):
                descriptor = ParameterDescriptor(
                    description="Latitude of the GPS origin for map loading", read_only=True
                )
                parameter = updated_params.map_data.gps_origin.latitude
                self.node_.declare_parameter(
                    self.prefix_ + "map_data.gps_origin.latitude", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "map_data.gps_origin.longitude"):
                descriptor = ParameterDescriptor(
                    description="Longitude of the GPS origin for map loading", read_only=True
                )
                parameter = updated_params.map_data.gps_origin.longitude
                self.node_.declare_parameter(
                    self.prefix_ + "map_data.gps_origin.longitude", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "map_data.gps_origin.altitude"):
                descriptor = ParameterDescriptor(
                    description="Altitude of the GPS origin for map loading", read_only=True
                )
                parameter = updated_params.map_data.gps_origin.altitude
                self.node_.declare_parameter(
                    self.prefix_ + "map_data.gps_origin.altitude", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "window_manager.planning_time"):
                descriptor = ParameterDescriptor(
                    description="Time allocated for planning (seconds)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.2
                descriptor.floating_point_range[-1].to_value = 5.0
                parameter = updated_params.window_manager.planning_time
                self.node_.declare_parameter(
                    self.prefix_ + "window_manager.planning_time", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "window_manager.buffer_time"):
                descriptor = ParameterDescriptor(
                    description="Buffer time for safety (seconds)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 2.0
                parameter = updated_params.window_manager.buffer_time
                self.node_.declare_parameter(
                    self.prefix_ + "window_manager.buffer_time", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "window_manager.lookahead_points"):
                descriptor = ParameterDescriptor(
                    description="Number of points to look ahead in the path", read_only=True
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 2
                descriptor.integer_range[-1].to_value = 20
                parameter = updated_params.window_manager.lookahead_points
                self.node_.declare_parameter(
                    self.prefix_ + "window_manager.lookahead_points", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "centerline_processor.interpolation_method"
            ):
                descriptor = ParameterDescriptor(
                    description="Interpolation method for pre-processing centerline points (linear or bezier)",
                    read_only=True,
                )
                parameter = updated_params.centerline_processor.interpolation_method
                self.node_.declare_parameter(
                    self.prefix_ + "centerline_processor.interpolation_method",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(self.prefix_ + "centerline_processor.target_spacing"):
                descriptor = ParameterDescriptor(
                    description="Target spacing between processed centerline points (meters)",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.5
                descriptor.floating_point_range[-1].to_value = 5.0
                parameter = updated_params.centerline_processor.target_spacing
                self.node_.declare_parameter(
                    self.prefix_ + "centerline_processor.target_spacing", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "centerline_processor.spacing_tolerance"
            ):
                descriptor = ParameterDescriptor(
                    description="Tolerance for acceptable deviation from target spacing (meters)",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.01
                descriptor.floating_point_range[-1].to_value = 0.5
                parameter = updated_params.centerline_processor.spacing_tolerance
                self.node_.declare_parameter(
                    self.prefix_ + "centerline_processor.spacing_tolerance", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "centerline_processor.bezier_window_size"
            ):
                descriptor = ParameterDescriptor(
                    description="Number of points in each window for bezier curve fitting",
                    read_only=True,
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 3
                descriptor.integer_range[-1].to_value = 20
                parameter = updated_params.centerline_processor.bezier_window_size
                self.node_.declare_parameter(
                    self.prefix_ + "centerline_processor.bezier_window_size", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "centerline_processor.bezier_overlap"):
                descriptor = ParameterDescriptor(
                    description="Number of points to overlap between bezier windows",
                    read_only=True,
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 10
                parameter = updated_params.centerline_processor.bezier_overlap
                self.node_.declare_parameter(
                    self.prefix_ + "centerline_processor.bezier_overlap", parameter, descriptor
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.trajectory_point_count"
            ):
                descriptor = ParameterDescriptor(
                    description="Number of equidistant points to be in optimized trajectory",
                    read_only=True,
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 50
                descriptor.integer_range[-1].to_value = 500
                parameter = updated_params.trajectory_optimization.trajectory_point_count
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.trajectory_point_count",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.base_tangent_factor"
            ):
                descriptor = ParameterDescriptor(
                    description="Base scaling factor for spline tangent vectors", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 2.0
                parameter = updated_params.trajectory_optimization.base_tangent_factor
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.base_tangent_factor",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.optimization_time_limit"
            ):
                descriptor = ParameterDescriptor(
                    description="Maximum time allowed for optimization (seconds)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 5.0
                parameter = updated_params.trajectory_optimization.optimization_time_limit
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.optimization_time_limit",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.arc_length_calculation_method"
            ):
                descriptor = ParameterDescriptor(
                    description="Method for calculating point spacing along spline (linear: faster but less accurate, simpson: more accurate but computationally intensive)",
                    read_only=True,
                )
                parameter = updated_params.trajectory_optimization.arc_length_calculation_method
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.arc_length_calculation_method",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.collision_checking.safety_margin"
            ):
                descriptor = ParameterDescriptor(
                    description="Safety margin for collision checking (meters)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.05
                descriptor.floating_point_range[-1].to_value = 1.0
                parameter = updated_params.trajectory_optimization.collision_checking.safety_margin
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.collision_checking.safety_margin",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_
                + "trajectory_optimization.collision_checking.collision_check_interval"
            ):
                descriptor = ParameterDescriptor(
                    description="Stride for collision checking (check every Nth point)",
                    read_only=True,
                )
                descriptor.integer_range.append(IntegerRange())
                descriptor.integer_range[-1].from_value = 1
                descriptor.integer_range[-1].to_value = 10
                parameter = (
                    updated_params.trajectory_optimization.collision_checking.collision_check_interval
                )
                self.node_.declare_parameter(
                    self.prefix_
                    + "trajectory_optimization.collision_checking.collision_check_interval",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.rprop.initial_step_size"
            ):
                descriptor = ParameterDescriptor(
                    description="Initial step size i.e delta_0 for RPROP optimization",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.01
                descriptor.floating_point_range[-1].to_value = 1.0
                parameter = updated_params.trajectory_optimization.rprop.initial_step_size
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.rprop.initial_step_size",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.rprop.minimum_step_size"
            ):
                descriptor = ParameterDescriptor(
                    description="Minimum step size i.e delta_min for RPROP optimization",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 1e-06
                descriptor.floating_point_range[-1].to_value = 0.01
                parameter = updated_params.trajectory_optimization.rprop.minimum_step_size
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.rprop.minimum_step_size",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.rprop.maximum_step_size"
            ):
                descriptor = ParameterDescriptor(
                    description="Maximum step size i.e delta_max for RPROP optimization",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 1.0
                descriptor.floating_point_range[-1].to_value = 100.0
                parameter = updated_params.trajectory_optimization.rprop.maximum_step_size
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.rprop.maximum_step_size",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.rprop.increase_factor"
            ):
                descriptor = ParameterDescriptor(
                    description="Increase factor i.e eta_plus for RPROP optimization",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 1.0
                descriptor.floating_point_range[-1].to_value = 2.0
                parameter = updated_params.trajectory_optimization.rprop.increase_factor
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.rprop.increase_factor",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(
                self.prefix_ + "trajectory_optimization.rprop.decrease_factor"
            ):
                descriptor = ParameterDescriptor(
                    description="Decrease factor i.e eta_minus for RPROP optimization",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 0.9
                parameter = updated_params.trajectory_optimization.rprop.decrease_factor
                self.node_.declare_parameter(
                    self.prefix_ + "trajectory_optimization.rprop.decrease_factor",
                    parameter,
                    descriptor,
                )

            if not self.node_.has_parameter(self.prefix_ + "distance_map.resolution"):
                descriptor = ParameterDescriptor(
                    description="Resolution of distance map grid (meters)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.05
                descriptor.floating_point_range[-1].to_value = 0.5
                parameter = updated_params.distance_map.resolution
                self.node_.declare_parameter(
                    self.prefix_ + "distance_map.resolution", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "distance_map.window_size"):
                descriptor = ParameterDescriptor(
                    description="Size of local window for distance calculations (meters). Controls the physical dimensions of the area considered for distance mapping.",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 1.0
                descriptor.floating_point_range[-1].to_value = 10.0
                parameter = updated_params.distance_map.window_size
                self.node_.declare_parameter(
                    self.prefix_ + "distance_map.window_size", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "distance_map.use_sobel"):
                descriptor = ParameterDescriptor(
                    description="Whether to use Sobel operator for gradient calculation",
                    read_only=True,
                )
                parameter = updated_params.distance_map.use_sobel
                self.node_.declare_parameter(
                    self.prefix_ + "distance_map.use_sobel", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "distance_map.sobel_threshold"):
                descriptor = ParameterDescriptor(
                    description="Minimum gradient magnitude threshold for normalization. Gradients below this value are considered noise.",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 1e-09
                descriptor.floating_point_range[-1].to_value = 0.001
                parameter = updated_params.distance_map.sobel_threshold
                self.node_.declare_parameter(
                    self.prefix_ + "distance_map.sobel_threshold", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.max_velocity"):
                descriptor = ParameterDescriptor(
                    description="Maximum velocity (m/s)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 3.0
                parameter = updated_params.robot_constraints.max_velocity
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.max_velocity", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.max_acceleration"):
                descriptor = ParameterDescriptor(
                    description="Maximum acceleration (m/s²)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 2.0
                parameter = updated_params.robot_constraints.max_acceleration
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.max_acceleration", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.max_deceleration"):
                descriptor = ParameterDescriptor(
                    description="Maximum deceleration (m/s²)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 3.0
                parameter = updated_params.robot_constraints.max_deceleration
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.max_deceleration", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.max_jerk"):
                descriptor = ParameterDescriptor(description="Maximum jerk (m/s³)", read_only=True)
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 5.0
                parameter = updated_params.robot_constraints.max_jerk
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.max_jerk", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.max_lateral_accel"):
                descriptor = ParameterDescriptor(
                    description="Maximum lateral acceleration (m/s²)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 1.0
                parameter = updated_params.robot_constraints.max_lateral_accel
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.max_lateral_accel", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.wheel_base"):
                descriptor = ParameterDescriptor(
                    description="Wheelbase of the robot (meters)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.1
                descriptor.floating_point_range[-1].to_value = 2.0
                parameter = updated_params.robot_constraints.wheel_base
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.wheel_base", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "robot_constraints.min_turning_radius"):
                descriptor = ParameterDescriptor(
                    description="Minimum turning radius (meters)", read_only=True
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = 0.5
                descriptor.floating_point_range[-1].to_value = 5.0
                parameter = updated_params.robot_constraints.min_turning_radius
                self.node_.declare_parameter(
                    self.prefix_ + "robot_constraints.min_turning_radius", parameter, descriptor
                )

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "map_data.map_path")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.map_data.map_path = param.value
            param = self.node_.get_parameter(self.prefix_ + "map_data.gps_origin.latitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.map_data.gps_origin.latitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "map_data.gps_origin.longitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.map_data.gps_origin.longitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "map_data.gps_origin.altitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.map_data.gps_origin.altitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "window_manager.planning_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.2, 5.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "window_manager.planning_time",
                    param.value,
                    "Invalid value set during initialization for parameter window_manager.planning_time: "
                    + validation_result,
                )
            updated_params.window_manager.planning_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "window_manager.buffer_time")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "window_manager.buffer_time",
                    param.value,
                    "Invalid value set during initialization for parameter window_manager.buffer_time: "
                    + validation_result,
                )
            updated_params.window_manager.buffer_time = param.value
            param = self.node_.get_parameter(self.prefix_ + "window_manager.lookahead_points")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 2, 20)
            if validation_result:
                raise InvalidParameterValueException(
                    "window_manager.lookahead_points",
                    param.value,
                    "Invalid value set during initialization for parameter window_manager.lookahead_points: "
                    + validation_result,
                )
            updated_params.window_manager.lookahead_points = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "centerline_processor.interpolation_method"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.one_of(param, ["linear", "bezier"])
            if validation_result:
                raise InvalidParameterValueException(
                    "centerline_processor.interpolation_method",
                    param.value,
                    "Invalid value set during initialization for parameter centerline_processor.interpolation_method: "
                    + validation_result,
                )
            updated_params.centerline_processor.interpolation_method = param.value
            param = self.node_.get_parameter(self.prefix_ + "centerline_processor.target_spacing")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.5, 5.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "centerline_processor.target_spacing",
                    param.value,
                    "Invalid value set during initialization for parameter centerline_processor.target_spacing: "
                    + validation_result,
                )
            updated_params.centerline_processor.target_spacing = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "centerline_processor.spacing_tolerance"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.01, 0.5)
            if validation_result:
                raise InvalidParameterValueException(
                    "centerline_processor.spacing_tolerance",
                    param.value,
                    "Invalid value set during initialization for parameter centerline_processor.spacing_tolerance: "
                    + validation_result,
                )
            updated_params.centerline_processor.spacing_tolerance = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "centerline_processor.bezier_window_size"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 3, 20)
            if validation_result:
                raise InvalidParameterValueException(
                    "centerline_processor.bezier_window_size",
                    param.value,
                    "Invalid value set during initialization for parameter centerline_processor.bezier_window_size: "
                    + validation_result,
                )
            updated_params.centerline_processor.bezier_window_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "centerline_processor.bezier_overlap")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1, 10)
            if validation_result:
                raise InvalidParameterValueException(
                    "centerline_processor.bezier_overlap",
                    param.value,
                    "Invalid value set during initialization for parameter centerline_processor.bezier_overlap: "
                    + validation_result,
                )
            updated_params.centerline_processor.bezier_overlap = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.trajectory_point_count"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 50, 500)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.trajectory_point_count",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.trajectory_point_count: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.trajectory_point_count = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.base_tangent_factor"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.base_tangent_factor",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.base_tangent_factor: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.base_tangent_factor = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.optimization_time_limit"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 5.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.optimization_time_limit",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.optimization_time_limit: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.optimization_time_limit = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.arc_length_calculation_method"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.one_of(param, ["linear", "simpson"])
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.arc_length_calculation_method",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.arc_length_calculation_method: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.arc_length_calculation_method = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.collision_checking.safety_margin"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.05, 1.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.collision_checking.safety_margin",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.collision_checking.safety_margin: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.collision_checking.safety_margin = param.value
            param = self.node_.get_parameter(
                self.prefix_
                + "trajectory_optimization.collision_checking.collision_check_interval"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1, 10)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.collision_checking.collision_check_interval",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.collision_checking.collision_check_interval: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.collision_checking.collision_check_interval = (
                param.value
            )
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.rprop.initial_step_size"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.01, 1.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.rprop.initial_step_size",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.rprop.initial_step_size: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.rprop.initial_step_size = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.rprop.minimum_step_size"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1e-06, 0.01)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.rprop.minimum_step_size",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.rprop.minimum_step_size: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.rprop.minimum_step_size = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.rprop.maximum_step_size"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1.0, 100.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.rprop.maximum_step_size",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.rprop.maximum_step_size: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.rprop.maximum_step_size = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.rprop.increase_factor"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1.0, 2.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.rprop.increase_factor",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.rprop.increase_factor: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.rprop.increase_factor = param.value
            param = self.node_.get_parameter(
                self.prefix_ + "trajectory_optimization.rprop.decrease_factor"
            )
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 0.9)
            if validation_result:
                raise InvalidParameterValueException(
                    "trajectory_optimization.rprop.decrease_factor",
                    param.value,
                    "Invalid value set during initialization for parameter trajectory_optimization.rprop.decrease_factor: "
                    + validation_result,
                )
            updated_params.trajectory_optimization.rprop.decrease_factor = param.value
            param = self.node_.get_parameter(self.prefix_ + "distance_map.resolution")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.05, 0.5)
            if validation_result:
                raise InvalidParameterValueException(
                    "distance_map.resolution",
                    param.value,
                    "Invalid value set during initialization for parameter distance_map.resolution: "
                    + validation_result,
                )
            updated_params.distance_map.resolution = param.value
            param = self.node_.get_parameter(self.prefix_ + "distance_map.window_size")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1.0, 10.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "distance_map.window_size",
                    param.value,
                    "Invalid value set during initialization for parameter distance_map.window_size: "
                    + validation_result,
                )
            updated_params.distance_map.window_size = param.value
            param = self.node_.get_parameter(self.prefix_ + "distance_map.use_sobel")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.distance_map.use_sobel = param.value
            param = self.node_.get_parameter(self.prefix_ + "distance_map.sobel_threshold")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 1e-09, 0.001)
            if validation_result:
                raise InvalidParameterValueException(
                    "distance_map.sobel_threshold",
                    param.value,
                    "Invalid value set during initialization for parameter distance_map.sobel_threshold: "
                    + validation_result,
                )
            updated_params.distance_map.sobel_threshold = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.max_velocity")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 3.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.max_velocity",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.max_velocity: "
                    + validation_result,
                )
            updated_params.robot_constraints.max_velocity = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.max_acceleration")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.max_acceleration",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.max_acceleration: "
                    + validation_result,
                )
            updated_params.robot_constraints.max_acceleration = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.max_deceleration")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 3.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.max_deceleration",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.max_deceleration: "
                    + validation_result,
                )
            updated_params.robot_constraints.max_deceleration = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.max_jerk")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 5.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.max_jerk",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.max_jerk: "
                    + validation_result,
                )
            updated_params.robot_constraints.max_jerk = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.max_lateral_accel")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 1.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.max_lateral_accel",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.max_lateral_accel: "
                    + validation_result,
                )
            updated_params.robot_constraints.max_lateral_accel = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.wheel_base")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.1, 2.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.wheel_base",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.wheel_base: "
                    + validation_result,
                )
            updated_params.robot_constraints.wheel_base = param.value
            param = self.node_.get_parameter(self.prefix_ + "robot_constraints.min_turning_radius")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, 0.5, 5.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "robot_constraints.min_turning_radius",
                    param.value,
                    "Invalid value set during initialization for parameter robot_constraints.min_turning_radius: "
                    + validation_result,
                )
            updated_params.robot_constraints.min_turning_radius = param.value

            self.update_internal_params(updated_params)
