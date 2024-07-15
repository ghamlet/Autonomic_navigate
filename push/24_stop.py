import math
import tf
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import Vector3Stamped, Point, PointStamped
import rospy
from clover import srv
from std_srvs.srv import Trigger
from sensor_msgs.msg import Range
from mavros_msgs.srv import CommandBool
from clover.srv import SetLEDEffect

rospy.init_node('flight')

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)
arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

ar = "aruco_49" # ID нарушителя
tfBuffer = tf2_ros.Buffer()
listener = tf2_ros.TransformListener(tfBuffer)

h_chl = 1# Для будущего кода дальномера

def navigate_wait(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

platform_found = False
def navigate_wait_fixed(x=0, y=0, z=1, yaw=float('nan'), speed=0.5, frame_id='aruco_map', auto_arm=False, tolerance=0.2):
    global platform_found
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)

    while not rospy.is_shutdown():
        platform_found = False
        telem_platform = get_telemetry(frame_id=ar)
        if str(telem_platform.x) != 'nan':
            print(str(telem_platform.x))
            platform_found = True
            break

        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.5)

def range_callback(msg):
    global h_chl
    h_chl = msg.range # Вывод Z по дальномеру

rospy.Subscriber('rangefinder/range', Range, range_callback)

def get_aruco_pose(frame_id): # Возвращает координаты, с помощью преобразование нового кадра
    global tfBuffer, listener
    try:
        trans = tfBuffer.lookup_transform(frame_id, ar, rospy.Time())
    except:
        return None
    pnt_l0 = tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(x=0, y=0, z=0)), trans)
    l0 = np.array([pnt_l0.point.x, pnt_l0.point.y, pnt_l0.point.z])
    return l0

def get_body_pose(frame_id): # Возвращает координаты, с помощью преобразование нового кадра 
    global tfBuffer, listener
    try:
        trans = tfBuffer.lookup_transform(frame_id, "body", rospy.Time())
    except:
        return None
    pnt_l0 = tf2_geometry_msgs.do_transform_point(PointStamped(point=Point(x=0, y=0, z=0)), trans)
    l0 = np.array([pnt_l0.point.x, pnt_l0.point.y, pnt_l0.point.z])
    return l0

def remove_0_vel(vel): # Скорости полета в POSITION
    if np.linalg.norm(vel[:2]) < 0.045:
        vel[0] = 0
        vel[1] = 0
    return vel


def go_to_home():
    set_effect(effect="rainbow")
    navigate_wait(z=0.4, speed=1, frame_id="body", auto_arm = True)
    navigate_wait(z=0.4, speed=1, frame_id="aruco_41")
    land()


#go_to_home()


z = 1

set_effect(r=255)
navigate_wait(z=1, speed=1, frame_id="body", auto_arm = True)
rospy.sleep(0.5)
set_effect(g=255, effect='blink')
navigate_wait(frame_id="aruco_24", z=0.5)

platform_found = False

while True:
    telem_platform = get_telemetry(frame_id=ar)
    if str(telem_platform) != 'nan':
        platform_found = True
        break
    rospy.sleep(0.5)

navigate_wait(x=0, y=0, z=z, speed=1.5, frame_id=ar, yaw=float('nan'), tolerance=0.2)
land()