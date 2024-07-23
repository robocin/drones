from geometry_msgs.msg import PoseArray

def PoseArray_to_list(pose_array):
    pose_list = []
    for pose in pose_array.poses:
        pose_list.append([pose.position.x, pose.position.y])
    return pose_list

def get_PoseArray_message(topic):
    import rospy

    vectors = rospy.wait_for_message(topic, PoseArray)
    vectors = PoseArray_to_list(vectors)

    if len(vectors) <= 0:    
        return None
    
    vectors = sorted(vectors, key=lambda x: x[0] ** 2 + x[1] ** 2)
    center_vector = vectors[0]

    return center_vector