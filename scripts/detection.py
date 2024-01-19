#!/usr/bin/env python3
# -*-coding: utf-8 -*-

from jetson_inference import detectNet
from jetson_utils import videoSource, videoOutput

import rospy
from zed_interfaces.msg import ObjectsStamped, Object
from std_msgs.msg import Header

from pyorbbecsdk import Pipeline
from pyorbbecsdk import Config
from pyorbbecsdk import OBSensorType, OBFormat
from pyorbbecsdk import OBError
import numpy as np

import coordinate_transfor

input = videoSource("/dev/video0",  options={'width': 640, 'height': 480, 'framerate': 30, 'input-codec':"mjpeg"})
output = videoOutput()	
net = detectNet("ssd-mobilenet-v2", threshold=0.5)

if __name__ == "__main__":

    rospy.init_node("Hello")
    detection_pub = rospy.Publisher('detected_objects', ObjectsStamped, queue_size=3)

    config = Config()
    pipeline = Pipeline()
    try:
        profile_list = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        assert profile_list is not None
        try:
            depth_profile = profile_list.get_video_stream_profile(640, 0, OBFormat.Y16, 30)
        except OBError as e:
            print("Error: ", e)
            depth_profile = profile_list.get_default_video_stream_profile()
        assert depth_profile is not None
        print("depth profile: ", type(depth_profile))
        config.enable_stream(depth_profile)
    except Exception as e:
        print(e)
    pipeline.start(config)

    objects_stamped_msg = ObjectsStamped()

    while True:
        img = input.Capture()
        if img is None: 
            continue  
        detections = net.Detect(img)
        
        frames = pipeline.wait_for_frames(5)
        if frames is None:
            continue
        depth_frame = frames.get_depth_frame()
        if depth_frame is None:
            continue
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        scale = depth_frame.get_depth_scale()

        depth_data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        depth_data = depth_data.reshape((height, width))

        depth_data = depth_data.astype(np.float32) * scale
        depth_data = depth_data.astype(np.uint16)
        
        # 将Object消息列表赋值给ObjectsStamped消息的objects字段
        objects_stamped_msg.header = Header()
        objects_stamped_msg.objects = coordinate_transfor.parse_to_object_detection(detections, depth_data)
        detection_pub.publish(objects_stamped_msg)

        output.Render(img)
        output.SetStatus("Objdetect | Network {:.0f} FPS".format(net.GetNetworkFPS()))
        if not input.IsStreaming() or not output.IsStreaming():
            break

    
