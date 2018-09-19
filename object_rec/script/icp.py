#!/usr/bin/env python
'''icp ROS Node'''
import rospy
import time
import roslaunch
from std_msgs.msg import String
from jsk_recognition_msgs.msg import Int32Stamped
from jsk_recognition_msgs.msg import ClassificationResult
from jsk_recognition_msgs.msg import ICPResult
from collections import deque
from collections import Counter
import message_filters as MF

icp_threshold = 0.7
cluster_num = 0
fifo_size = 10
package = "nodelet"
node_type = "nodelet"
icp_name = "icp"
icp_args = "load jsk_pcl/ICPRegistration object_recognition_manager"

transform_name = "transform"
transform_args = "load jsk_pcl_utils/TfTransformCloud object_recognition_manager"

class_fifos = []
icp_subs = []
target_names = ['cube','cuboid','cylinder']

def cluster_callback(data):
    '''cluster Callback Function'''
    global cluster_num
    cluster_num =  data.data

def icp_callback(*args):
    '''icp Callback Function'''
    global cluster_num
    global fifo_size
    global icp_threshold
    global icp_name
    global class_fifos
    global target_names

    icp_result = ClassificationResult()
    icp_result.header.frame_id = args[0].header.frame_id
    icp_result.classifier = icp_name
    icp_result.header.stamp = rospy.Time.now()
    icp_result.target_names = target_names

    for i in range(0, cluster_num):
        class_fifos[i].popleft()
        class_fifos[i].append(args[i].name)
        count = Counter(class_fifos[i])
        mode = int(count.most_common(1)[0][0])
        freq = count.most_common(1)[0][1]
        prob = float(freq) / fifo_size
        icp_result.labels.append(mode)
        icp_result.label_names.append(target_names[mode])
        icp_result.label_proba.append(prob)
        #rospy.loginfo("Mode %s with freq %f from ICP %d",mode,prob,i)
    
    icp_pub.publish(icp_result)

def run_node(num):
    global package
    global node_type
    global icp_name
    global icp_args
    global transform_name
    global transform_args
    global launch
    global fifo_size
    global icp_subs
    global icp_pub
    global class_fifos

    _icp_name = icp_name + str(num)
    _transform_name = transform_name + str(num)
    
    icp_remap_args = [("~input","/"+_transform_name+"/output"),("~input_reference_array","/pcd_to_pointcloud/output")] 
    icp_node = roslaunch.core.Node(package,node_type,name=_icp_name,args=icp_args,remap_args=icp_remap_args)

    transform_remap_args = [("~input","/cluster_decomposer_final/output0"+str(num))] 
    transform_node = roslaunch.core.Node(package,node_type,name=_transform_name,args=transform_args,remap_args=transform_remap_args)

    icp_process = launch.launch(icp_node)
    transform_process = launch.launch(transform_node)

    rospy.set_param('/'+_icp_name+'/transform_3dof', True)
    rospy.set_param('/'+_icp_name+'/use_flipped_initial_pose', False)
    rospy.set_param('/'+_icp_name+'/algorithm', 'ICP')
    rospy.set_param('/'+_icp_name+'/euclidean_fittness_epsilon', 0.001)
    rospy.set_param('/'+_icp_name+'/max_iteration', 1000)
    rospy.set_param('/'+_icp_name+'/correspondence_distance', 3)

    rospy.set_param('/'+_transform_name+'/target_frame_id', 'table00')
    
    class_list = []
    for x in range(0, fifo_size):
        class_list.append('0')

    class_fifo = deque(class_list)
    class_fifos.append(class_fifo)

    icp_subs.append(MF.Subscriber('/'+_icp_name+'/icp_result', ICPResult))

def icp():
    '''icp shape recognition node'''  
    global cluster_num
    global icp_subs
    global launch
    global icp_pub

    rospy.init_node('icp', anonymous=False)
    cluster_num_old = 0

    rospy.Subscriber("/euclidean_clustering/cluster_num", Int32Stamped, cluster_callback)
    icp_pub = rospy.Publisher('~output',ClassificationResult, queue_size=10)

    launch = roslaunch.scriptapi.ROSLaunch()
    launch.start()
    
    r = rospy.Rate(0.5)
    while not rospy.is_shutdown():
        if (cluster_num > cluster_num_old):
            for x in range(cluster_num_old, cluster_num):
                run_node(x)
            cluster_num_old = cluster_num
            sync = MF.ApproximateTimeSynchronizer(icp_subs, queue_size=100, slop=0.1)
            sync.registerCallback(icp_callback)
        r.sleep()

    # spin() simply keeps python from exiting until this node is stopped

    rospy.spin()

if __name__ == '__main__':
    icp()

"""    <node name="tf_transform_cloud"
          pkg="nodelet" type="nodelet"
          args="load jsk_pcl_utils/TfTransformCloud $(arg MANAGER)">
      <remap from="~input" to="cluster_decomposer_final/output00"/>
      <rosparam subst_value="true">
        target_frame_id: table00
      </rosparam>
    </node>"""

"""<node name="icp_registration"
        pkg="nodelet" type="nodelet"
        args="load jsk_pcl/ICPRegistration $(arg MANAGER)">
    <remap from="~input" to="tf_transform_cloud/output"/>
    <remap from="~input_reference_array" to="/pcd_to_pointcloud/output"/>
    <rosparam>
      transform_3dof: true
      use_flipped_initial_pose: false
      algorithm: ICP
      euclidean_fittness_epsilon: 0.001
      max_iteration: 1000
      correspondence_distance: 2
    </rosparam>
</node>"""
