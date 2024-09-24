I built this in the following environment:
- ROS 2 Humble
- Ubuntu Jammy (22.04)

## To build
`colcon build --packages-select perception_interfaces`

Then perception_interfaces should be discoverable to other ros 2 packages.