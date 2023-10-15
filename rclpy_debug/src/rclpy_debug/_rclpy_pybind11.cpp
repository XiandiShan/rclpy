// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <pybind11/pybind11.h>

#include <rcl/domain_id.h>
#include <rcl/time.h>
#include <rcl_action/types.h>

#include <rmw/qos_profiles.h>
#include <rmw/time.h>

#include "action_client.hpp"
#include "action_goal_handle.hpp"
#include "action_server.hpp"
#include "client.hpp"
#include "clock.hpp"
#include "context.hpp"
#include "destroyable.hpp"
#include "duration.hpp"
#include "clock_event.hpp"
#include "exceptions.hpp"
#include "graph.hpp"
#include "guard_condition.hpp"
#include "lifecycle.hpp"
#include "logging.hpp"
#include "logging_api.hpp"
#include "names.hpp"
#include "node.hpp"
#include "publisher.hpp"
#include "qos.hpp"
#include "qos_event.hpp"
#include "serialization.hpp"
#include "service.hpp"
#include "service_info.hpp"
#include "signal_handler.hpp"
#include "subscription.hpp"
#include "time_point.hpp"
#include "timer.hpp"
#include "utils.hpp"
#include "wait_set.hpp"

namespace py = pybind11;

PYBIND11_MODULE(_rclpy_pybind11, m) {
  m.doc() = "ROS 2 Python client library.";

  rclpy_debug::define_destroyable(m);

  py::enum_<rcl_clock_type_t>(m, "ClockType")
  .value("UNINITIALIZED", RCL_CLOCK_UNINITIALIZED)
  .value("ROS_TIME", RCL_ROS_TIME)
  .value("SYSTEM_TIME", RCL_SYSTEM_TIME)
  .value("STEADY_TIME", RCL_STEADY_TIME);

  py::enum_<rcl_action_goal_event_t>(m, "GoalEvent")
  .value("EXECUTE", GOAL_EVENT_EXECUTE)
  .value("CANCEL_GOAL", GOAL_EVENT_CANCEL_GOAL)
  .value("SUCCEED", GOAL_EVENT_SUCCEED)
  .value("ABORT", GOAL_EVENT_ABORT)
  .value("CANCELED", GOAL_EVENT_CANCELED);

  m.attr("RCL_DEFAULT_DOMAIN_ID") = py::int_(RCL_DEFAULT_DOMAIN_ID);
  m.attr("RMW_DURATION_INFINITE") = py::int_(rmw_time_total_nsec(RMW_DURATION_INFINITE));

  py::enum_<rcl_clock_change_t>(m, "ClockChange")
  .value(
    "ROS_TIME_NO_CHANGE", RCL_ROS_TIME_NO_CHANGE,
    "ROS time is active and will continue to be active")
  .value(
    "ROS_TIME_ACTIVATED", RCL_ROS_TIME_ACTIVATED,
    "ROS time is being activated")
  .value(
    "ROS_TIME_DEACTIVATED", RCL_ROS_TIME_DEACTIVATED,
    "ROS TIME is being deactivated, the clock will report system time after the jump")
  .value(
    "SYSTEM_TIME_NO_CHANGE", RCL_SYSTEM_TIME_NO_CHANGE,
    "ROS time is inactive and the clock will keep reporting system time");

  py::enum_<rmw_qos_compatibility_type_t>(m, "QoSCompatibility")
  .value("OK", RMW_QOS_COMPATIBILITY_OK)
  .value("WARNING", RMW_QOS_COMPATIBILITY_WARNING)
  .value("ERROR", RMW_QOS_COMPATIBILITY_ERROR);

  py::class_<rclpy_debug::QoSCheckCompatibleResult>(
    m, "QoSCheckCompatibleResult",
    "Result type for checking QoS compatibility with result")
  .def(py::init<>())
  .def_readonly("compatibility", &rclpy_debug::QoSCheckCompatibleResult::compatibility)
  .def_readonly("reason", &rclpy_debug::QoSCheckCompatibleResult::reason);

  py::register_exception<rclpy_debug::RCUtilsError>(m, "RCUtilsError", PyExc_RuntimeError);
  py::register_exception<rclpy_debug::RMWError>(m, "RMWError", PyExc_RuntimeError);
  auto rclerror = py::register_exception<rclpy_debug::RCLError>(m, "RCLError", PyExc_RuntimeError);
  py::register_exception<rclpy_debug::RCLInvalidROSArgsError>(
    m, "RCLInvalidROSArgsError", rclerror.ptr());
  py::register_exception<rclpy_debug::UnknownROSArgsError>(m, "UnknownROSArgsError", PyExc_RuntimeError);
  py::register_exception<rclpy_debug::NodeNameNonExistentError>(
    m, "NodeNameNonExistentError", rclerror.ptr());
  py::register_exception<rclpy_debug::UnsupportedEventTypeError>(
    m, "UnsupportedEventTypeError", rclerror.ptr());
  py::register_exception<rclpy_debug::NotImplementedError>(
    m, "NotImplementedError", PyExc_NotImplementedError);
  py::register_exception<rclpy_debug::InvalidHandle>(
    m, "InvalidHandle", PyExc_RuntimeError);

  rclpy_debug::define_client(m);

  rclpy_debug::define_context(m);

  rclpy_debug::define_duration(m);

  rclpy_debug::define_publisher(m);

  rclpy_debug::define_service(m);

  rclpy_debug::define_service_info(m);

  m.def(
    "rclpy_qos_check_compatible", &rclpy_debug::qos_check_compatible,
    "Check if two QoS profiles are compatible.");

  rclpy_debug::define_action_client(m);
  rclpy_debug::define_action_goal_handle(m);
  rclpy_debug::define_action_server(m);
  m.def(
    "rclpy_action_get_rmw_qos_profile", &rclpy_debug::rclpy_action_get_rmw_qos_profile,
    "Get an action RMW QoS profile.");
  rclpy_debug::define_guard_condition(m);
  rclpy_debug::define_timer(m);
  rclpy_debug::define_subscription(m);
  rclpy_debug::define_time_point(m);
  rclpy_debug::define_clock(m);
  rclpy_debug::define_waitset(m);

  m.def(
    "rclpy_expand_topic_name", &rclpy_debug::expand_topic_name,
    "Expand a topic name.");
  m.def(
    "rclpy_remap_topic_name", &rclpy_debug::remap_topic_name,
    "Remap a topic name.");
  m.def(
    "rclpy_get_validation_error_for_topic_name", &rclpy_debug::get_validation_error_for_topic_name,
    "Get the error message and invalid index of a topic name or None if valid.");
  m.def(
    "rclpy_get_validation_error_for_full_topic_name",
    &rclpy_debug::get_validation_error_for_full_topic_name,
    "Get the error message and invalid index of a full topic name or None if valid.");
  m.def(
    "rclpy_get_validation_error_for_namespace", &rclpy_debug::get_validation_error_for_namespace,
    "Get the error message and invalid index of a namespace or None if valid.");
  m.def(
    "rclpy_get_validation_error_for_node_name", &rclpy_debug::get_validation_error_for_node_name,
    "Get the error message and invalid index of a node name or None if valid.");
  m.def(
    "rclpy_resolve_name", &rclpy_debug::resolve_name,
    "Expand and remap a topic or service name.");

  m.def(
    "rclpy_get_topic_names_and_types",
    &rclpy_debug::graph_get_topic_names_and_types,
    "Get all topic names and types in the ROS graph.");
  m.def(
    "rclpy_get_publisher_names_and_types_by_node",
    &rclpy_debug::graph_get_publisher_names_and_types_by_node,
    "Get topic names and types for which a remote node has publishers.");
  m.def(
    "rclpy_get_subscriber_names_and_types_by_node",
    &rclpy_debug::graph_get_subscriber_names_and_types_by_node,
    "Get topic names and types for which a remote node has subscribers.");
  m.def(
    "rclpy_get_publishers_info_by_topic",
    &rclpy_debug::graph_get_publishers_info_by_topic,
    "Get publishers info for a topic.");
  m.def(
    "rclpy_get_subscriptions_info_by_topic",
    &rclpy_debug::graph_get_subscriptions_info_by_topic,
    "Get subscriptions info for a topic.");
  m.def(
    "rclpy_get_service_names_and_types",
    &rclpy_debug::graph_get_service_names_and_types,
    "Get all service names and types in the ROS graph.");
  m.def(
    "rclpy_get_service_names_and_types_by_node",
    &rclpy_debug::graph_get_service_names_and_types_by_node,
    "Get service names and types for which a remote node has servers.");
  m.def(
    "rclpy_get_client_names_and_types_by_node",
    &rclpy_debug::graph_get_client_names_and_types_by_node,
    "Get service names and types for which a remote node has clients.");

  m.def(
    "rclpy_serialize", &rclpy_debug::serialize,
    "Serialize a ROS message.");
  m.def(
    "rclpy_deserialize", &rclpy_debug::deserialize,
    "Deserialize a ROS message.");

  rclpy_debug::define_node(m);
  rclpy_debug::define_qos_event(m);

  m.def(
    "rclpy_get_rmw_implementation_identifier",
    &rclpy_debug::get_rmw_implementation_identifier,
    "Retrieve the identifier for the active RMW implementation.");

  m.def(
    "rclpy_assert_liveliness", &rclpy_debug::assert_liveliness,
    "Assert the liveliness of an entity.");

  m.def(
    "rclpy_remove_ros_args", &rclpy_debug::remove_ros_args,
    "Remove ROS-specific arguments from argument vector.");

  rclpy_debug::define_rmw_qos_profile(m);

  m.def(
    "rclpy_logging_fini", rclpy_debug::logging_fini,
    "Finalize RCL logging.");
  m.def(
    "rclpy_logging_configure", rclpy_debug::logging_configure,
    "Initialize RCL logging.");

  rclpy_debug::define_logging_api(m);
  rclpy_debug::define_signal_handler_api(m);
  rclpy_debug::define_clock_event(m);
  rclpy_debug::define_lifecycle_api(m);
}
