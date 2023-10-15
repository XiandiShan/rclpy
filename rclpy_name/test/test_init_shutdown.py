# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import pytest
import rclpy_debug
from rclpy_debug import signals
from rclpy.exceptions import NotInitializedException
from rclpy.signals import SignalHandlerOptions


def test_init():
    context = rclpy_debug.context.Context()
    rclpy_debug.init(context=context)
    rclpy_debug.shutdown(context=context)


def test_init_with_unknown_ros_args():
    from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy

    context = rclpy_debug.context.Context()
    unknown_ros_args_error_pattern = r'\[\'unknown\'\]'
    with pytest.raises(_rclpy.UnknownROSArgsError, match=unknown_ros_args_error_pattern):
        rclpy_debug.init(context=context, args=['--ros-args', 'unknown'])


def test_init_with_non_utf8_arguments():
    context = rclpy_debug.context.Context()
    # Embed non decodable characters e.g. due to
    # wrong locale settings.
    # See PEP-383 for further reference.
    args = ['my-node.py', 'Ragnar\udcc3\udcb6k']
    with pytest.raises(UnicodeEncodeError):
        rclpy_debug.init(context=context, args=args)


def test_init_shutdown_sequence():
    context = rclpy_debug.context.Context()
    rclpy_debug.init(context=context)
    rclpy_debug.shutdown(context=context)
    context = rclpy_debug.context.Context()  # context cannot be reused but should not interfere
    rclpy_debug.init(context=context)
    rclpy_debug.shutdown(context=context)

    # global
    rclpy_debug.init()
    rclpy_debug.shutdown()
    rclpy_debug.init()
    rclpy_debug.shutdown()


def test_double_init():
    context = rclpy_debug.context.Context()
    rclpy_debug.init(context=context)
    try:
        with pytest.raises(RuntimeError):
            rclpy_debug.init(context=context)
    finally:
        rclpy_debug.shutdown(context=context)


def test_double_shutdown():
    context = rclpy_debug.context.Context()
    rclpy_debug.init(context=context)
    rclpy_debug.shutdown(context=context)
    with pytest.raises(RuntimeError):
        rclpy_debug.shutdown(context=context)


def test_create_node_without_init():
    context = rclpy_debug.context.Context()
    with pytest.raises(NotInitializedException):
        rclpy_debug.create_node('foo', context=context)


def test_init_with_domain_id():
    rclpy_debug.init(domain_id=123)
    assert rclpy_debug.get_default_context().get_domain_id() == 123
    rclpy_debug.shutdown()
    context = rclpy_debug.context.Context()
    rclpy_debug.init(context=context, domain_id=123)
    assert context.get_domain_id() == 123
    rclpy_debug.shutdown(context=context)


def test_signal_handlers():
    rclpy_debug.init()
    assert SignalHandlerOptions.ALL == signals.get_current_signal_handlers_options()
    rclpy_debug.shutdown()
    assert SignalHandlerOptions.NO == signals.get_current_signal_handlers_options()


def test_init_with_invalid_domain_id():
    with pytest.raises(RuntimeError):
        rclpy_debug.init(domain_id=-1)
