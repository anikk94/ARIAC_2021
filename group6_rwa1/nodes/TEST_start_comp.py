#!/usr/bin/env python3
import rospy
from std_srvs.srv import Trigger



srv_name = '/ariac/start_competition'
rospy.wait_for_service(srv_name)
comp = rospy.ServiceProxy(srv_name, Trigger)

try:
    comp()
except rospy.ServiceException as exc:
    print(f'{srv_name} failed with error \n{exc}')
