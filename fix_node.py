import os
base = '/home/taiphoon/Taiphoon_Agent/src/nodes'

with open(base + '/basic_controller_node.cpp', 'r') as f:
    content = f.read()

old_odom = '''bool BasicControllerNode::initialize_odometry_subscriber()
{
    odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
        " fmu/out/vehicle_odometry,
