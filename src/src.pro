TEMPLATE = subdirs

SUBDIRS += \
    common \
    base \
    maps \
    planners \
    data \
    comms \
    commsMACE \
    data_generic_item \
    data_generic_state_item \
    data_generic_command_item \
    mace_core \
    commsMAVLINK \
    commsMACEHelper \
    base_topic \
    data_generic_item_topic \
    data_generic_state_item_topic \
    data_generic_command_item_topic \
    data_generic_mission_item_topic \
    data_interface_MAVLINK \
    data_interface_MACE \
    data_vehicle_sensors \
    controllers \
    module_vehicle_generic \
    module_vehicle_MAVLINK \
    module_vehicle_ardupilot \
    module_external_link\
    module_ground_station \
    module_path_planning_NASAPhase2 \
    module_vehicle_sensors \
    voropp \
    module_resource_task_allocation \
    module_ROS \
    mace \
    TestMaps \
    module_generic_MAVLINK


base.depends = common
maps.depends = base
planners.depends = maps
data.depends = common
baseTopic.depends = base
comms.depends = common
commsMACE.depends = data
data_generic_item.depends = data
data_generic_state_item.depends = data_generic_item
data_generic_command_item.depends = data_generic_state_item
mace_core.depends = data_generic_command_item
commsMAVLINK.depends = mace_core
commsMACEHelper.depends = mace_core
data_generic_item_topic.depends = data_generic_item
data_generic_state_item_topic.depends = data_generic_state_item
data_generic_command_item_topic.depends = data_generic_state_item_topic
data_generic_mission_item_topic.depends = data_generic_command_item_topic
data_interface_MAVLINK.depends = data_generic_mission_item_topic
data_interface_MACE.depends = data_generic_mission_item_topic
data_vehicle_sensors.depends = data_generic_state_item_topic
controllers.depends = data_interface_MACE
module_generic_MAVLINK.depends = controllers
module_vehicle_generic.depends = controllers
module_vehicle_MAVLINK.depends = module_generic_MAVLINK
module_vehicle_ardupilot.depends = module_vehicle_MAVLINK
module_external_link.depends = module_vehicle_ardupilot
module_ground_station.depends = module_external_link
module_path_planning_NASAPhase2.depends = module_ground_station
module_vehicle_sensors.depends = module_path_planning_NASAPhase2
voropp.depends = module_vehicle_sensors
module_resource_task_allocation.depends = voropp
module_ROS.depends = module_resource_task_allocation
mace.depends = module_ROS

