#!/usr/bin/env python3

import rospy
from nist_gear.srv import AGVToAssemblyStation


srv_name = '/ariac/agv2/submit_shipment'
print(srv_name)
assembly_station_name = "as2"
print(assembly_station_name)
shipment_type = "order_0_kitting_shipment_0"
print(shipment_type)
rospy.wait_for_service(srv_name)

move_agv = rospy.ServiceProxy(srv_name, AGVToAssemblyStation)

try:
    move_agv(assembly_station_name, shipment_type)
except rospy.ServiceException as exc:
    print(f'{srv_name} failed with error \n{exc}')