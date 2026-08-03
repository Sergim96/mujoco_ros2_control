# `hardware_interface` diagnostic overlay

This package is the ROS 2 Lyrical `hardware_interface` 6.7.1 source used by
this workspace. It is kept at the same package version and ABI so the Lyrical
`controller_manager` and `joint_limits` binaries remain compatible.

The local change makes command-limit errors name the constraint that actually
clamped the command—position, velocity, acceleration, or effort/torque—and
prints the evaluated value and allowed range.

For joints with separate position and velocity command interfaces, position
commands are checked only against hard position bounds. Velocity limits are
applied to the velocity command directly, rather than deriving velocity from
the change between consecutive position commands. Position-only joints retain
the standard ROS position-step velocity limiter.

Build the workspace with `--allow-overriding hardware_interface`, then source
the workspace `install/setup.bash` before launching ROS nodes.
