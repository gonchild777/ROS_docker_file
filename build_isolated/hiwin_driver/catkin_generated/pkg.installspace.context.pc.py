# generated from catkin/cmake/template/pkg.context.pc.in
CATKIN_PACKAGE_PREFIX = ""
PROJECT_PKG_CONFIG_INCLUDE_DIRS = "${prefix}/include;/home/ROS/workspace/devel_isolated/hiwin_robot_client_library/include".split(';') if "${prefix}/include;/home/ROS/workspace/devel_isolated/hiwin_robot_client_library/include" != "" else []
PROJECT_CATKIN_DEPENDS = "controller_manager;hardware_interface;pluginlib;pass_through_controllers;roscpp;std_srvs".replace(';', ' ')
PKG_CONFIG_LIBRARIES_WITH_PREFIX = "-lhiwin_driver_plugin;/home/ROS/workspace/devel_isolated/hiwin_robot_client_library/lib/libhrsdk.so".split(';') if "-lhiwin_driver_plugin;/home/ROS/workspace/devel_isolated/hiwin_robot_client_library/lib/libhrsdk.so" != "" else []
PROJECT_NAME = "hiwin_driver"
PROJECT_SPACE_DIR = "/home/ROS/workspace/install_isolated"
PROJECT_VERSION = "0.0.0"
