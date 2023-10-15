Execution and Callbacks
=======================

There are two components that control the execution of callbacks: **executors** and **callback groups**.

Executors are responsible for the actual execution of callbacks and should extend the :class:`.Executor` class.

Callback groups are used to enforce concurrency rules for callbacks and should extend the :class:`.CallbackGroup` class.

Executors
---------

.. automodule:: rclpy_debug.executors


Callback Groups
---------------
.. automodule:: rclpy_debug.callback_groups
