from zed_interfaces.msg import  Object

# 类别信息
with open('/home/cothink/catkin_ws/src/jetson-inference/scripts/ssd_coco_labels.txt', 'r', encoding='utf-8') as file:
    lines = file.read().splitlines()

# 内参信息
width = 640
height = 480
fx = 543.9993896484375
fy = 543.9993896484375
cx = 314.0804748535156
cy = 242.5907745361328

def pixels_to_meters(pixel_x, pixel_y, depth_value):
    # 计算相机坐标系中的归一化坐标
    normalized_x = (pixel_x - cx) / fx
    normalized_y = (pixel_y - cy) / fy

    # 使用深度信息将归一化坐标转换为以米为单位的坐标
    x_meter = normalized_x * depth_value
    y_meter = normalized_y * depth_value

    return x_meter, y_meter

def parse_to_object_detection(detections, depth_data):

    # 创建一个Object消息列表，用于存储每个目标的信息
    objects_msg_list = []
    for detection in detections:
        # 获取检测结果的标签和置信度
        label = lines[detection.ClassID]
        confidence = detection.Confidence
        # 获取检测框的位置信息[x, y]
        center_xy = detection.Center
        # 将深度值从毫米转换为米
        depth_value = depth_data[int(center_xy[1]), int(center_xy[0])] / 1000.0  
        if depth_value < 0.6  or depth_value > 6:
            continue
        
        # 将像素坐标转换为以米为单位的坐标
        x_meter, y_meter = pixels_to_meters(center_xy[0], center_xy[1], depth_value)
        #print(x_meter, y_meter, depth_value)

        # 创建一个Object消息并填充信息
        object_msg = Object()
        #object_msg.label = label
        object_msg.sublabel = label  # 根据需要设置子标签
        object_msg.confidence = confidence
        object_msg.position = [x_meter, y_meter, depth_value]

        # 将Object消息添加到列表中
        objects_msg_list.append(object_msg)

    return  objects_msg_list

def parse_to_pose_estimation(poses, depth_data):
    
    # 创建一个Object消息列表，用于存储每个目标的信息
    objects_msg_list = []
    for pose in poses:
        neck_coords = None
        left_hip_coords = None
        right_hip_coords = None

        for keypoint in pose.Keypoints:
            if keypoint.ID == 17:  # 17 表示neck
                neck_coords = [keypoint.x, keypoint.y]
            if keypoint.ID == 11:  # 11 表示left_hip
                left_hip_coords = [keypoint.x, keypoint.y]
            if keypoint.ID == 12:  # 12 表示right_hip
                right_hip_coords = [keypoint.x, keypoint.y]

        # 检查左肩和右肩是否都存在
        if neck_coords is not None and left_hip_coords is not None and right_hip_coords is not None:
            # 将深度值从毫米转换为米
            leftr_middle = [(neck_coords[0]+left_hip_coords[0])/2 , (neck_coords[1]+left_hip_coords[1])/2]
            right_middle = [(neck_coords[0]+right_hip_coords[0])/2 , (neck_coords[1]+right_hip_coords[1])/2]
            center_coords =[( leftr_middle[0]+right_middle[0])/2,  ( leftr_middle[1]+right_middle[1])/2 ]
            depth_value = depth_data[int(center_coords[1]),  int(center_coords[0])] / 1000.0
        else:
            continue

        if depth_value < 0.6  or depth_value > 6:
            continue

        # 将像素坐标转换为以米为单位的坐标
        x_meter, y_meter = pixels_to_meters(center_coords[0], center_coords[1], depth_value)
        print(x_meter, y_meter, depth_value)
        
        # 创建一个Object消息并填充信息
        object_msg = Object()
        #object_msg.label = label
        object_msg.sublabel = "person"  # 根据需要设置子标签
        #object_msg.confidence = confidence
        object_msg.position = [x_meter, y_meter, depth_value]

        # 将Object消息添加到列表中
        objects_msg_list.append(object_msg)

    return  objects_msg_list
    