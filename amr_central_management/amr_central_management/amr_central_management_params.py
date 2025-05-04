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


class amr_central_management:
    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        use_time_based_routing = False

        class __GpsOrigin:
            latitude = 50.7153508
            longitude = 10.4680332
            altitude = 0.0

        gps_origin = __GpsOrigin()

    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = amr_central_management.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("amr_central_management." + prefix)

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
                if param.name == self.prefix_ + "gps_origin.latitude":
                    validation_result = ParameterValidators.bounds(param, -90.0, 90.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.gps_origin.latitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "gps_origin.longitude":
                    validation_result = ParameterValidators.bounds(param, -180.0, 180.0)
                    if validation_result:
                        return SetParametersResult(successful=False, reason=validation_result)
                    updated_params.gps_origin.longitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "gps_origin.altitude":
                    updated_params.gps_origin.altitude = param.value
                    self.logger_.debug(
                        param.name + ": " + param.type_.name + " = " + str(param.value)
                    )

                if param.name == self.prefix_ + "use_time_based_routing":
                    updated_params.use_time_based_routing = param.value
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
            if not self.node_.has_parameter(self.prefix_ + "gps_origin.latitude"):
                descriptor = ParameterDescriptor(
                    description="Latitude of the GPS origin for coordinate transformation and map loading",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = -90.0
                descriptor.floating_point_range[-1].to_value = 90.0
                parameter = updated_params.gps_origin.latitude
                self.node_.declare_parameter(
                    self.prefix_ + "gps_origin.latitude", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "gps_origin.longitude"):
                descriptor = ParameterDescriptor(
                    description="Longitude of the GPS origin for coordinate transformation and map loading",
                    read_only=True,
                )
                descriptor.floating_point_range.append(FloatingPointRange())
                descriptor.floating_point_range[-1].from_value = -180.0
                descriptor.floating_point_range[-1].to_value = 180.0
                parameter = updated_params.gps_origin.longitude
                self.node_.declare_parameter(
                    self.prefix_ + "gps_origin.longitude", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "gps_origin.altitude"):
                descriptor = ParameterDescriptor(
                    description="Altitude of the GPS origin for coordinate transformation and map loading",
                    read_only=True,
                )
                parameter = updated_params.gps_origin.altitude
                self.node_.declare_parameter(
                    self.prefix_ + "gps_origin.altitude", parameter, descriptor
                )

            if not self.node_.has_parameter(self.prefix_ + "use_time_based_routing"):
                descriptor = ParameterDescriptor(
                    description="Whether to use time-based path calculation. Currently only constant speed on all road types is implemented.",
                    read_only=True,
                )
                parameter = updated_params.use_time_based_routing
                self.node_.declare_parameter(
                    self.prefix_ + "use_time_based_routing", parameter, descriptor
                )

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "gps_origin.latitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, -90.0, 90.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "gps_origin.latitude",
                    param.value,
                    "Invalid value set during initialization for parameter gps_origin.latitude: "
                    + validation_result,
                )
            updated_params.gps_origin.latitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "gps_origin.longitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            validation_result = ParameterValidators.bounds(param, -180.0, 180.0)
            if validation_result:
                raise InvalidParameterValueException(
                    "gps_origin.longitude",
                    param.value,
                    "Invalid value set during initialization for parameter gps_origin.longitude: "
                    + validation_result,
                )
            updated_params.gps_origin.longitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "gps_origin.altitude")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.gps_origin.altitude = param.value
            param = self.node_.get_parameter(self.prefix_ + "use_time_based_routing")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.use_time_based_routing = param.value

            self.update_internal_params(updated_params)
