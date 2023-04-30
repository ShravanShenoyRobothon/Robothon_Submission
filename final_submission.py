#!/usr/bin/env python3

import cv2
import pyrealsense2 as rs
#from realsense_depth import *
import torch
import numpy as np
import json
import math
import cv2.aruco as aruco
from control_msgs.msg import JointTrajectoryControllerState 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import rospy
#import mediapipe as mp 
#import imutils
import sys
import math
from dh_gripper_msgs.msg import GripperCtrl
import roslib 
roslib.load_manifest('ur_driver')
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from std_msgs.msg import String
import time
import pyttsx3


voice = pyttsx3.init()


offset_x = 0
offset_y = 0
z_error = 0
gripper_pos = 0
seq_no = 0
z_tab = 9
board_angle_error = 18

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


class_id_to_name_dict = {0:'door', 1:'door_knob', 2:'micro_controller', 3:'photo_int_unit', 4:'probe', 5:'probe_end', 6:'probe_holder', 7:'probe_start', 8:'project_board', 9:'slider', 11:'task_end', 10:'task_start'}
#class_id_to_name_dict = {0:'door', 1:'door_knob', 2: 'marker', 3:'micro_controller', 4:'photo_int_unit', 5:'probe', 6:'probe_end', 7:'probe_holder', 8:'probe_start', 9:'project_board', 10:'slider', 12:'task_end', 11:'task_start'}
class_with_depth = {}
class_with_uv = {}
class_with_xyz = {}
class_with_state = {}

# Create a pipeline
pipeline = rs.pipeline()

# Create a config and configure the pipeline to stream
#  different resolutions of color and depth streams
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
profile = pipeline.start(config)

# Getting the depth sensor's depth scale (see rs-align example for explanation)
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth Scale is: " , depth_scale)



align_to = rs.stream.color
align = rs.align(align_to)


## Functions
def talk(audio):
    print(audio)
    voice.say(audio)
    voice.runAndWait()



def start_pipeline_IROS():
    global pipeline, align
    try:
        pipeline.stop()
    except:
        pass
    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)



    align_to = rs.stream.color
    align = rs.align(align_to)

def gripper_open(force = 30, position = 750, speed = 0):
    gripper_publisher = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    gripper_msg = GripperCtrl()
    gripper_msg.position = position
    gripper_msg.force = force
    gripper_msg.speed = speed
    gripper_publisher.publish(gripper_msg)
    rospy.sleep(1)
    
def gripper_close(force = 25, position = 0, speed = 0):
    gripper_publisher = rospy.Publisher('/gripper/ctrl', GripperCtrl, queue_size=10)
    gripper_msg = GripperCtrl()
    gripper_msg.position = position
    gripper_msg.force = force
    gripper_msg.speed = speed
    gripper_publisher.publish(gripper_msg)
    rospy.sleep(1)

def findArucoMarker_IROS(img,markerSize=4,totalMarkers=250,draw=True):
    global pipeline, align
    global aruco_angle_diff
    imgGray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    feature=getattr(aruco,f'DICT_{markerSize}X{markerSize}_{totalMarkers}')
    arucoDict=aruco.Dictionary_get(feature)
    arucoParam=aruco.DetectorParameters_create()
    bboxs,ids,rejected=aruco.detectMarkers(imgGray,arucoDict,parameters=arucoParam)
    aruco_center = [0,0]
    if draw:
        num_aruco_detected = len(bboxs)
        if num_aruco_detected > 0:
            print(bboxs[0])
            aruco.drawDetectedMarkers(img,[bboxs[0]])
            [x,y] = [int((sum(bboxs[0][0])[0])/4),int((sum(bboxs[0][0])[1])/4)]
            # print([x,y])
            aruco_center = [x,y]
            cv2.circle(img,(x,y),7,(0,0,255),-1)
            [[x1,y1],[x2,y2]] = [[((bboxs[0][0][0][0]+bboxs[0][0][1][0])/2),((bboxs[0][0][0][1]+bboxs[0][0][1][1])/2)],[((bboxs[0][0][2][0]+bboxs[0][0][3][0])/2),((bboxs[0][0][2][1]+bboxs[0][0][3][1])/2)]]
            # print([x1,y1,x2,y2])
            aruco_angle_diff = math.degrees(math.atan2((y1-y2),(x1-x2))) + 90
            # print(aruco_angle_diff)
            aruco_angle = (-aruco_angle_diff) + state[0]
            if aruco_angle<(-180):
                aruco_angle = aruco_angle + 360
            

        
    return [bboxs,ids, aruco_center,aruco_angle]


def Link(th,d,a,alp):
    Tth=np.array([[math.cos(th),-math.sin(th),0,0],[math.sin(th),math.cos(th),0,0],[0,0,1,0],[0,0,0,1]])
    Td=np.array([[1,0,0,0],[0,1,0,0],[0,0,1,d],[0,0,0,1]])
    Ta=np.array([[1,0,0,a],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
    Talp=np.array([[1,0,0,0],[0,math.cos(alp),-math.sin(alp),0],[0,math.sin(alp),math.cos(alp),0],[0,0,0,1]])
    T=np.dot(Tth,np.dot(Td,np.dot(Ta,Talp)))
    return(T)

def LinkSerialRobot(A):
    s=len(A)
    Robot=A[s-1]
    for i in range (1,s):
        Robot=np.dot(A[s-(i+1)],Robot)
    return(Robot)


def frame_transform(x,y,z, state):
    #print("State inside FT= ", state)
    [t1,t2,t3,t4,t5,t6] = np.deg2rad(state)
    #print("State inside frame transform", state)
    #L1 = Link(t1, 89.159, 0, math.pi/2)  ## all angles in radians
    L1 = Link(t1, 100.0, 0, math.pi/2)
    L2 = Link(t2, 135.85, -425.0, 0)
    L3 = Link(t3, -119.7, -392.25, 0)
    L4 = Link(t4, 93.0, 0, math.pi/2)
    L5 = Link(t5, 94.65, 0, -math.pi/2)
    L6 = Link(t6, 82.3, 0, math.pi)
    pos = np.array([[1,0,0,x],[0,1,0,y+100],[0,0,1,-(z+30)],[0,0,0,1]])
    FK=LinkSerialRobot([L1,L2,L3,L4,L5,L6])
    FK = np.dot(FK,pos)

    x = int((FK[0][3])/10)
    y = int((FK[1][3])/10)
    z = int((FK[2][3])/10) 
    # print("Transformed_pos = ", transformed_pos)

    return x,y,z

def my_ur5_ik_geo(x, y, z, th4 = 0, th5 = 0, th6 = 0, move = False, delay = 2.5):
    
    global state
    # Known  robot parameters (all dimensions in cm)
    l1 = 8.916
    l2 = 42.5
    l3 = 39.225
    l5 = 9.465
    l6 = 8.23 + 20.0
    d2 = 13.585
    d3 = 11.97
    d4 = 9.3
    
    # end effector orientation parameters to be defined by user (angles in degrees)
    th4_ = th4  #angle of joint 4 wrt world  # +ve to look front  # -ve to look behind
    th5_ = th5  #angle of joint 5 wrt world
    th6_ = th6  #angle of joint 6 wrt world
    
    if th6_ < (-180):
        th6_ = th6_ + 360
    
    # algorithm start
    
    # end effector offset part
    dee = l6 * math.sin(math.radians(th5_))
    lee = (l5 * math.cos(math.radians(th4_))) + ((l6 * math.cos(math.radians(th5_))) * math.sin(math.radians(th4_)))
    zee = (l6 * math.cos(math.radians(th5_))) * math.cos(math.radians(th4_)) - (l5 * math.sin(math.radians(th4_)))
    
    
    # triangle rule in first 3 links
    th1_ = math.degrees(math.asin(((d2 + d4) - (d3 + dee)) / math.sqrt((x * x) + (y * y))))
    l = math.sqrt((x * x) + (y * y) - (((d2 + d4) - (d3 + dee)) * ((d2 + d4) - (d3 + dee)))) - lee
    th1 = math.degrees(math.atan2(y, x)) + th1_
    c = math.sqrt((l * l) + ((z + zee -l1) * (z + zee -l1)))
    alp = math.degrees(math.atan2((z + zee - l1), l))
    th2_ = math.degrees(math.acos(((l2 * l2) + (c * c) - (l3 * l3)) / (2 * l2 * c)))
    th2 = th2_ + alp
    th3_ = math.degrees(math.acos(((l2 * l2) + (l3 * l3) - (c * c)) / (2 * l2 * l3)))
    th3 = th3_ - 90
    th4 = (180 - (th2 + th3_)) + th4_
    th5 = th5_
    th6 = th6_
    
    # callibration
    if th1<-90:
        t1 = th1 + 360
    else:
        t1 = th1
    t2 = th2 - 180
    t3 = th3 - 90
    t4 = th4 - 90
    t5 = th5 + 90
    t6 = th6
    
    state = [t1,t2,t3,t4,t5,t6]
    if move == True:
        move_robot(state, delay)
    
    return(state)

def move_robot(state, delay = 1.5, wait = True):
    [t1,t2,t3,t4,t5,t6] = state
    
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    goal_position = [math.radians(float(t1)), math.radians(float(t2)), math.radians(float(t3)), math.radians(float(t4)), math.radians(float(t5)), math.radians(float(t6))]
    
    g.trajectory.points = [
        JointTrajectoryPoint(positions=goal_position, velocities=[0]*6, time_from_start=rospy.Duration(delay)),
        # JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
        # JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
        # JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))
        ]
    client.send_goal(g)
    if wait == True:
        try:
            client.wait_for_result()
        except KeyboardInterrupt:
            client.cancel_goal()
            raise
    #print("\n \n The present position is: ", [t1, t2, t3, t4, t5, t6])
    return [t1, t2, t3, t4, t5, t6]

    
def arucoAug(bbox, id, img, drawId = True):
    tl = bbox[0][0][0], bbox[0][0][1]
    x=int(bbox[0][0][0])
    y=int(bbox[0][0][1])

    if drawId:
        cv2.putText(img,str(id),(x,y),cv2.FONT_HERSHEY_PLAIN,1,(255,0,255),2)
    return img   


def detect_aruco_IROS():
    global pipeline, align
    # Get frameset of color and depth
    # Get frameset of color and depth
    frames = pipeline.wait_for_frames()
    # frames.get_depth_frame() is a 640x360 depth image

    # Align the depth frame to color frame
    aligned_frames = align.process(frames)

    # Get aligned frames
    aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
    color_frame = aligned_frames.get_color_frame()

    # Validate that both frames are valid
    # if not aligned_depth_frame or not color_frame:
    #     continue

    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())
    frame = color_image
    frame = cv2.resize(frame,(1240,960))
    while True:
        arucofound=findArucoMarker_IROS(frame)
         # loop through all the markers and augment each one
        # if  len(arucofound[0])!=0:
        #     for bbox, id in zip(arucofound[0], arucofound[1]):
        #         frame = arucoAug(bbox, id, frame)
        aruco_center = arucofound[2]
        aruco_angle = arucofound[3]
        print("aruco angle = ",aruco_angle)
        
        [x,y] = [int(aruco_center[0]/2), int(aruco_center[1]/2)]
        z = depth_image[y][x]
        if z != 0:
            x_c = int((x - 320) * (255 / 320) * (z / 500))  #converting to dimension in mm
            y_c = int(((480 - y) - 240) * (195 / 240) * (z / 500))
            z_c = int(z)
            pos_in_cam = [x_c, y_c, z_c]
            [x_w,y_w,z_w] = frame_transform(x_c, y_c, z_c, state)
            transformed_pos = [x_w,y_w,z_w]
            transformed_aruco_center = [transformed_pos[0],transformed_pos[1]]
        cv2.imshow("Image",frame)
        key=cv2.waitKey(0)  
        if key==ord("q"):
        # if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
            break  
    return(transformed_aruco_center,aruco_angle)

def detect_objects():
    global class_id_to_name_dict, class_with_depth, class_with_uv, class_with_xyz, class_with_state, depth_scale, depth_sensor, model
    class_id_to_name_dict = {0:'door', 1:'door_knob', 2:'micro_controller', 3:'photo_int_unit', 4:'probe', 5:'probe_end', 6:'probe_holder', 7:'probe_start', 8:'project_board', 9:'slider', 11:'task_end', 10:'task_start'}
    #class_id_to_name_dict = {0:'door', 1:'door_knob', 2: 'marker', 3:'micro_controller', 4:'photo_int_unit', 5:'probe', 6:'probe_end', 7:'probe_holder', 8:'probe_start', 9:'project_board', 10:'slider', 12:'task_end', 11:'task_start'}
    class_with_depth = {}
    class_with_uv = {}
    class_with_xyz = {}
    class_with_state = {}
    #Hardcoded z values
    # class_with_z_value = {'door':platform_height + 86.5, 'door_knob':platform_height + 89.5, 'marker': platform_height + 85, 'micro_controller':platform_height + 102, 'photo_int_unit':platform_height + 97, 'probe':platform_height + 100, 'probe_end':platform_height + 85, 'probe_holder':platform_height + 100, 'probe_start':platform_height + 90, 'project_board':platform_height + 85, 'slider':platform_height + 88, 'task_end':platform_height + 86, 'task_start':platform_height + 86}
    

    model = torch.hub.load('.','custom', '/home/ril/YOLOv7-custom/yolov7/yolov7_custom.pt', source='local')

    while True:
        frames = pipeline.wait_for_frames()

        aligned_frames = align.process(frames)

        aligned_depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        #depth_image = np.asanyarray(colorizer.colorize(aligned_depth_frame).get_data())
        depth_image = np.asanyarray(aligned_depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        #images = np.hstack((color_image, depth_image))
        cv2.namedWindow('Realsense', cv2.WINDOW_AUTOSIZE)
        frame = color_image
        frame = cv2.resize(frame, (640, 480))
        
        predictions = model(frame)
        for pred in predictions.pred:
            for i in range(len(pred)):
                font = cv2.FONT_HERSHEY_COMPLEX
                pred_individual = np.array(pred.cpu()[i], dtype=float)
                u1, v1, u2, v2, conf, class_id = pred_individual
                u1, v1, u2, v2, conf, class_id = int(u1), int(v1), int(u2), int(v2), int(conf*100), int(class_id)
                center_u, center_v = int(u1+(u2-u1)/2), int(v1 + (v2-v1)/2)
                cv2.circle(color_image, (int(center_u), int(center_v)), 2, (0,255,255), 2)
                cv2.rectangle(color_image, (int(u1), int(v1)), (u2, v2), (0, 255, 0), 2)
                class_name = class_id_to_name_dict.get(class_id)
                #print("Class bbox", class_name, bboxes)
                label_size = cv2.getTextSize(class_name, font, 0.25, 1)[0]
                text_u = u1 + int(((u2-u1)-label_size[0])/2)
                text_v = v1 - int(label_size[1]*1.5)
                cv2.putText(color_image, class_name, (text_u, text_v), font, 0.5, (0, 255, 255), 2)
                #cv2.putText(color_image, str(conf), (text_u + label_size[0], text_v), font, 0.5, (0, 0, 255), 2)
                class_with_depth[class_name] = depth_image[center_v, center_u]
                class_with_uv[class_name] = (center_u, center_v)
                cx_r = center_u * 0.96875
                cy_r = center_v
                cz_r =  depth_image[center_v][center_u]
                #print(cz_r)
                x_r = int((cx_r - 320) * (255 / 320) * (cz_r / 500))
                y_r = int(((480 - cy_r) - 240) * (195 / 240) * (cz_r / 500))
                z_r = int(cz_r)
                x_real, y_real, z_real = frame_transform(x_r, y_r, z_r, state=state)
                class_with_xyz[class_name] = (x_real, y_real, z_real)
        cv2.imshow("Image", color_image)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows() 
            break

    


try:    
    rospy.init_node("my_ur5_trajectory_publisher", anonymous=True, disable_signals=True)
    client = actionlib.SimpleActionClient('/scaled_pos_joint_traj_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    print ("Waiting for server...")
    client.wait_for_server()
    print ("Connected to server")
except KeyboardInterrupt:
    rospy.signal_shutdown("KeyboardInterrupt")
    raise

gripper_open()
state = my_ur5_ik_geo(-20,30,55,th4=0, move = True, delay= 4)
gripper_close()

print('Ready to beign')
talk('Ready to begin')

print("Detecting_aruco ...")
transformed_aruco_center,th_aruco_board = detect_aruco_IROS()
print("Center of ArUco is at : ",transformed_aruco_center)
talk('Finding and aligning to the board')
print("Moving to ArUco center ...")
[x,y] = [(transformed_aruco_center[0]+((math.cos((math.pi)/4))*10)), (transformed_aruco_center[1]-((math.sin((math.pi)/4))*10))]
if (((x*x) + (y*y))> 0) :
    try:
        th6 = ((np.rad2deg(math.atan2(y,x))-th_aruco_board))
        print("th6 = ",th6)
        state = my_ur5_ik_geo(x,y, 25,th6 = ((np.rad2deg(math.atan2(y,x))-th_aruco_board)), move = True)
        rospy.sleep(1)
    except:
        print("Unable to solve IK")
        talk('Unable to solve IK')
        pass
print("Detecting_aruco ...")
transformed_aruco_center, th_aruco_board = detect_aruco_IROS()
# board_position = [x,y,25]
board_angle = aruco_angle_diff
# state[5] = aruco_angle_diff
print(f'state[5] = {state[5]}')

state[5] = state[5] + aruco_angle_diff
print(f'state[5] = {state[5]}')
print(f'aruco_angle_diff = {aruco_angle_diff}')
print(f'th_aruco_board = {th_aruco_board}')
print("Alligning")
move_robot(state)
board_looking_state = state
board_orientation = state[0] + state[5]
transformed_aruco_center,th_aruco_board = detect_aruco_IROS()
print("Done")
talk("Done")
rospy.sleep(2)


talk('Detecting objects')
detect_objects()

print(class_with_xyz)

# Begin task 1
talk('task 1')
[x,y,z] = class_with_xyz['task_start']
[x,y,z] = [x-1.8, y-1.0, z_tab]
print('task_start_coordinate : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
state = my_ur5_ik_geo(x,y,z)
state[5] = board_orientation - state[0]
move_robot(state, delay = 1)
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 1)
time.sleep(1)
state = board_looking_state
move_robot(state)




# Begin task 2
time.sleep(1)
talk('task 2')

detect_objects()
print(class_with_xyz)
gripper_open(position=150)
[x,y,z] = class_with_xyz['slider']
[x,y,z] = [x-2.3, y-0.5, z_tab]
print('slider_coordinate : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
state = my_ur5_ik_geo(x,y,z+0.5)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
gripper_close(position=30)
time.sleep(1)

[sliding_offset_x, sliding_offset_y] = [(3.2 * math.cos(math.radians(180 - (board_orientation-board_angle_error)))), (3.2 * math.sin(math.radians(180 - (board_orientation-board_angle_error))))] 
state = my_ur5_ik_geo(x+sliding_offset_x,y-sliding_offset_y,z+0.5)
state[5] = board_orientation - state[0]
move_robot(state, delay = 7)

state = my_ur5_ik_geo(x,y,z+0.5)
state[5] = board_orientation - state[0]
move_robot(state, delay = 7)

gripper_open(position=150)
time.sleep(1)

state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
time.sleep(1)
state = board_looking_state
move_robot(state)



#Begin task 3
time.sleep(1)
talk('task 3')

detect_objects()
print(class_with_xyz)
gripper_open(position=200)
[x,y,z] = class_with_xyz['probe_start']
[x,y,z] = [x-1.8, y-0.8, z_tab]
print('probe_start : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
state = my_ur5_ik_geo(x,y,z+0.5)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)

gripper_close(force = 25 ,position=30)
time.sleep(1)


state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
gripper_close(force = 5 ,position=88)
time.sleep(1)


[x,y,z] = class_with_xyz['probe_end']
[x,y,z] = [x-1.35, y-1.45, z_tab]
print('probe_end : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)

state = my_ur5_ik_geo(x,y,z+0.5)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)

gripper_open(position=150)

state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)


time.sleep(1)
state = board_looking_state
move_robot(state)


#Begin task 4
time.sleep(1)
talk('task 4')

detect_objects()
print(class_with_xyz)
gripper_open(position=380)
[x,y,z] = class_with_xyz['door_knob']
[x,y,z] = [x-0.5, y+0.2, z_tab]
[xi, yi, zi] = [x,y,z]
print('task_door_knob_coordinate : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3,th4=50)
move_robot(state, delay = 4)

state = my_ur5_ik_geo(x,y,z+0.7,th4=50)
move_robot(state, delay = 1)

time.sleep(1)
gripper_close(position = 110)
time.sleep(2)

# r = 6.5
r = 2
[x_off, y_off] = [(r * math.cos(math.radians(180 - (board_orientation-board_angle_error+5)))), (r * math.sin(math.radians(180 - (board_orientation-board_angle_error+5))))] 
state = my_ur5_ik_geo(x-x_off,y+y_off,z+3.2,th4=45)
move_robot(state, delay = 5)
# time.sleep(2)
r = 2.8
[x_off, y_off] = [(r * math.cos(math.radians(180 - (board_orientation-board_angle_error+5)))), (r * math.sin(math.radians(180 - (board_orientation-board_angle_error+5))))] 
state = my_ur5_ik_geo(x-x_off,y+y_off,z+4.7,th4=30)
move_robot(state, delay = 5)
# time.sleep(2)
r = 4
[x_off, y_off] = [(r * math.cos(math.radians(180 - (board_orientation-board_angle_error+5)))), (r * math.sin(math.radians(180 - (board_orientation-board_angle_error+5))))] 
state = my_ur5_ik_geo(x-x_off,y+y_off,z+5,th4=20)
move_robot(state, delay = 5)
# time.sleep(2)
r = 6.5
[x_off, y_off] = [(r * math.cos(math.radians(180 - (board_orientation-board_angle_error+5)))), (r * math.sin(math.radians(180 - (board_orientation-board_angle_error+5))))] 
state = my_ur5_ik_geo(x-x_off,y+y_off,z+5.8,th4=0)
move_robot(state, delay = 5)
# time.sleep(2)
r = 10.5
[x_off, y_off] = [(r * math.cos(math.radians(180 - (board_orientation-board_angle_error+5)))), (r * math.sin(math.radians(180 - (board_orientation-board_angle_error+5))))] 
state = my_ur5_ik_geo(x-x_off,y+y_off,z+4.5,th4=-20)
move_robot(state, delay = 5)
time.sleep(2)
    
time.sleep(1)
gripper_open(position = 350)
time.sleep(2)
state = my_ur5_ik_geo((x-x_off)-2,(y+y_off)+2,z+9, th4=-20)
move_robot(state, delay = 1)


time.sleep(1)
state = board_looking_state
state[0] = state[0]-10
state[5] = state[5]+10
move_robot(state,delay = 4)

# detect_objects()
# print(class_with_xyz)
gripper_open(position=220)
[x,y,z] = class_with_xyz['probe']
[x,y,z] = [x-0.5, y-0.2, z_tab]
print('probe : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3,th4=45)
move_robot(state, delay = 4)

state = my_ur5_ik_geo(x,y,z+0.2,th4=45)
move_robot(state, delay = 1)

gripper_close()

[x_off, y_off] = [(4 * math.cos(math.radians(180 - (board_orientation-board_angle_error)))), (4 * math.sin(math.radians(180 - (board_orientation-board_angle_error))))] 
state = my_ur5_ik_geo(x+x_off,y-y_off,z+0.2, th4 = 45)
move_robot(state, delay = 5)
time.sleep(2)

state = my_ur5_ik_geo(x+x_off,y-y_off,z+15, th4 = 45)
move_robot(state, delay = 5)
time.sleep(2)

state[5] = state[5] + 180
move_robot(state, delay = 3)
time.sleep(2)

state = my_ur5_ik_geo(xi-x_off,yi+y_off,z+12, th4 = 45)
state[5] = state[5] + 180
move_robot(state, delay = 5)
time.sleep(2)


gripper_open()
time.sleep(1)
state[5] = state[5]
move_robot(state, delay = 3)
time.sleep(2)

time.sleep(1)
state = board_looking_state
move_robot(state,delay = 4)






#Begin task 6
time.sleep(1)
talk('task 6')

# detect_objects()
# print(class_with_xyz)
gripper_close(position=0)
[x,y,z] = class_with_xyz['task_end']
[x,y,z] = [x-1.9, y, z_tab]
print('task_end_coordinate : ',[x,y,z])
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 2)
state = my_ur5_ik_geo(x,y,z)
state[5] = board_orientation - state[0]
move_robot(state, delay = 1)
state = my_ur5_ik_geo(x,y,z+3)
state[5] = board_orientation - state[0]
move_robot(state, delay = 1)

time.sleep(1)
state = board_looking_state
move_robot(state)

talk("Sequence completed")
state = my_ur5_ik_geo(-20,30,55,th4=0, move = True, delay= 4)
gripper_close()
