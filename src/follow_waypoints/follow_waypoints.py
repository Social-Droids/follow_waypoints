#!/usr/bin/env python

import threading
import rospy
import actionlib
from smach import State,StateMachine
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray ,PointStamped
from std_msgs.msg import Empty
from tf import TransformListener
import tf
import math
import rospkg
import csv
import time
from geometry_msgs.msg import PoseStamped
import dynamic_reconfigure.client

# change Pose to the correct frame 
def changePose(waypoint,target_frame):
    if waypoint.header.frame_id == target_frame:
        # already in correct frame
        return waypoint
    if not hasattr(changePose, 'listener'):
        changePose.listener = tf.TransformListener()
    tmp = PoseStamped()
    tmp.header.frame_id = waypoint.header.frame_id
    tmp.pose = waypoint.pose.pose
    try:
        changePose.listener.waitForTransform(
            target_frame, tmp.header.frame_id, rospy.Time(0), rospy.Duration(3.0))
        pose = changePose.listener.transformPose(target_frame, tmp)
        ret = PoseWithCovarianceStamped()
        ret.header.frame_id = target_frame
        ret.pose.pose = pose.pose
        return ret
    except:
        rospy.loginfo("CAN'T TRANSFORM POSE TO {} FRAME".format(target_frame))
        exit()


#Path for saving and retreiving the pose.csv file 
output_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
journey_file_path = rospkg.RosPack().get_path('follow_waypoints')+"/saved_path/pose.csv"
waypoints = []

class FollowPath(State):
    def __init__(self):
        global journey_file_path
        
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'])
        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')
        self.duration = rospy.get_param('~wait_duration', 0.0)
        # Get a move_base action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base...')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base.')
        rospy.loginfo('Starting a tf listner.')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()

        journey_file_path = rospy.get_param("~journey_file_path", output_file_path)

        self.actual_xy_goal_tolerance = rospy.get_param("~xy_goal_tolerance", 0.3)
        self.actual_yaw_goal_tolerance = rospy.get_param("~yaw_goal_tolerance", 3.14)

        # self.last_xy_goal_tolerance = rospy.get_param('/move_base/TebLocalPlannerROS/xy_goal_tolerance')
        # self.last_yaw_goal_tolerance = rospy.get_param('/move_base/TebLocalPlannerROS/yaw_goal_tolerance')
        # self.clientDR = dynamic_reconfigure.client.Client("move_base/TebLocalPlannerROS", timeout=30, config_callback=self.callbackDR)

        self.last_xy_goal_tolerance = rospy.get_param('/move_base/DWAPlannerROS/xy_goal_tolerance')
        self.last_yaw_goal_tolerance = rospy.get_param('/move_base/DWAPlannerROS/yaw_goal_tolerance')
        self.clientDR = dynamic_reconfigure.client.Client("move_base/DWAPlannerROS", timeout=30, config_callback=self.callbackDR)

    def callbackDR(self, config):
        rospy.loginfo("Navigation tolerance set to [xy_goal:{xy_goal_tolerance}, yaw_goal:{yaw_goal_tolerance}]".format(**config))

    def execute(self, userdata):

        self.clientDR.update_configuration({
            "xy_goal_tolerance":self.actual_xy_goal_tolerance, 
            "yaw_goal_tolerance":self.actual_yaw_goal_tolerance
            })

        global waypoints
        # Execute waypoints each in sequence
        for waypoint in waypoints:
            # Break if preempted
            if waypoints == []:
                rospy.loginfo('The waypoint queue has been reset.')
                break
            # Otherwise publish next waypoint as goal
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = waypoint.pose.pose.position
            goal.target_pose.pose.orientation = waypoint.pose.pose.orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %
                    (waypoint.pose.pose.position.x, waypoint.pose.pose.position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            self.client.send_goal(goal)

            self.client.wait_for_result()
            rospy.loginfo("Waiting for %f sec..." % self.duration)
            time.sleep(self.duration)

        self.clientDR.update_configuration({
            "xy_goal_tolerance":self.last_xy_goal_tolerance, 
            "yaw_goal_tolerance":self.last_yaw_goal_tolerance
            })

        return 'success'

def convert_PoseWithCovArray_to_PoseArray(waypoints):
    """Used to publish waypoints as pose array so that you can see them in rviz, etc."""
    poses = PoseArray()
    poses.header.frame_id = rospy.get_param('~goal_frame_id','map')
    poses.poses = [pose.pose.pose for pose in waypoints]
    return poses

class GetPath(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'], input_keys=['waypoints'], output_keys=['waypoints'])
        # Subscribe to pose message to get new waypoints
        self.addpose_topic = rospy.get_param('~addpose_topic','/initialpose')
        # Create publsher to publish waypoints as pose array so that you can see them in rviz, etc.
        self.posearray_topic = rospy.get_param('~posearray_topic','/waypoints')
        self.poseArray_publisher = rospy.Publisher(self.posearray_topic, PoseArray, queue_size=1)

        reset_thread = threading.Thread(target=self.wait_for_path_reset)
        reset_thread.start()

    # Start thread to listen for reset messages to clear the waypoint queue
    def wait_for_path_reset(self):
        """thread worker function"""
        global waypoints
        while not rospy.is_shutdown():
            data = rospy.wait_for_message('/path_reset', Empty)
            rospy.loginfo('Recieved path RESET message')
            self.initialize_path_queue()
            rospy.sleep(3) # Wait 3 seconds because `rostopic echo` latches
                            # for three seconds and wait_for_message() in a
                            # loop will see it again.

    def initialize_path_queue(self):
        global waypoints
        waypoints = [] # the waypoint queue
        # publish empty waypoint queue as pose array so that you can see them the change in rviz, etc.
        self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

    def wait_for_path_ready(self):
        """thread worker function"""
        data = rospy.wait_for_message('/path_ready', Empty)
        rospy.loginfo('Recieved path READY message')
        self.path_ready = True
        with open(output_file_path, 'w') as file:
            for current_pose in waypoints:
                file.write(str(current_pose.pose.pose.position.x) + ',' + str(current_pose.pose.pose.position.y) + ',' + str(current_pose.pose.pose.position.z) + ',' + str(current_pose.pose.pose.orientation.x) + ',' + str(current_pose.pose.pose.orientation.y) + ',' + str(current_pose.pose.pose.orientation.z) + ',' + str(current_pose.pose.pose.orientation.w)+ '\n')
        rospy.loginfo('poses written to '+ output_file_path)	

    def wait_for_start_journey(self):
            """
            Wait for a message on the /start_journey topic to signal that the robot should begin following the saved path.
            """
            global waypoints

            rospy.loginfo("Waiting for /start_journey message to follow saved path")

            start_journey = rospy.Subscriber('/start_journey', Empty, self.start_journey_callback)

            # Wait until start_journey_callback() sets self.start_journey_bool to True
            while not self.start_journey_bool:
                if rospy.is_shutdown():
                    start_journey.unregister()
                    return
                rospy.sleep(0.1)

            # Read saved poses from follow_waypoints/saved_path/poses.csv and follow them
            poses_file_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'saved_path', 'poses.csv')
            poses_file = open(poses_file_path, 'r')

            with poses_file:
                poses_reader = csv.reader(poses_file)
                next(poses_reader)  # skip header row

                for row in poses_reader:
                    if rospy.is_shutdown():
                        start_journey.unregister()
                        return

                    pose_stamped = PoseStamped()
                    pose_stamped.pose.position.x = float(row[0])
                    pose_stamped.pose.position.y = float(row[1])
                    pose_stamped.pose.position.z = float(row[2])
                    pose_stamped.pose.orientation.x = float(row[3])
                    pose_stamped.pose.orientation.y = float(row[4])
                    pose_stamped.pose.orientation.z = float(row[5])
                    pose_stamped.pose.orientation.w = float(row[6])

                    # check if the robot has reached the waypoint
                    while not self.is_goal_reached(pose_stamped):
                        if rospy.is_shutdown():
                            start_journey.unregister()
                            return
                        rospy.sleep(0.1)

                    # remove the reached waypoint from the CSV file
                    with open(poses_file_path, 'r') as input_file, open(poses_file_path + '.tmp', 'w') as output_file:
                        reader = csv.reader(input_file)
                        writer = csv.writer(output_file)

                        header_row = next(reader)
                        writer.writerow(header_row)

                        for r in reader:
                            if r != row:
                                writer.writerow(r)

                    os.rename(poses_file_path + '.tmp', poses_file_path)

                    rospy.loginfo("Reached waypoint: x=%f, y=%f, z=%f" % (pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z))


    def execute(self, userdata):
        global waypoints
        self.initialize_path_queue()
        self.path_ready = False

        # Start thread to listen for when the path is ready (this function will end then)
        # Also will save the clicked path to pose.csv file
        ready_thread = threading.Thread(target=self.wait_for_path_ready)
        ready_thread.start()

        self.start_journey_bool = False

        # Start thread to listen start_jorney 
        # for loading the saved poses from follow_waypoints/saved_path/poses.csv            
            
        start_journey_thread = threading.Thread(target=self.wait_for_start_journey)
        start_journey_thread.start()

        topic = self.addpose_topic;
        rospy.loginfo("Waiting to recieve waypoints via Pose msg on topic %s" % topic)
        rospy.loginfo("To start following waypoints: 'rostopic pub /path_ready std_msgs/Empty -1'")
        rospy.loginfo("OR")
        rospy.loginfo("To start following saved waypoints: 'rostopic pub /start_journey std_msgs/Empty -1'")


        # Wait for published waypoints or saved path  loaded
        while (not self.path_ready and not self.start_journey_bool):
            try:
                pose = rospy.wait_for_message(topic, PoseWithCovarianceStamped, timeout=1)
            except rospy.ROSException as e:
                if 'timeout exceeded' in str(e):
                    continue  # no new waypoint within timeout, looping...
                else:
                    raise e
            rospy.loginfo("Recieved new waypoint")
            waypoints.append(changePose(pose, "map"))
            # publish waypoint queue as pose array so that you can see them in rviz, etc.
            self.poseArray_publisher.publish(convert_PoseWithCovArray_to_PoseArray(waypoints))

        # Path is ready! return success and move on to the next state (FOLLOW_PATH)
        return 'success'


class PathComplete(State):
    def __init__(self):
        State.__init__(self, outcomes=['success'])

    def execute(self, userdata):
        rospy.loginfo('###############################')
        rospy.loginfo('##### REACHED FINISH GATE #####')
        rospy.loginfo('###############################')
        return 'success'

def main():
    rospy.init_node('follow_waypoints')

    sm = StateMachine(outcomes=['success'])

    with sm:
        StateMachine.add('GET_PATH', GetPath(),
                           transitions={'success':'FOLLOW_PATH'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('FOLLOW_PATH', FollowPath(),
                           transitions={'success':'PATH_COMPLETE'},
                           remapping={'waypoints':'waypoints'})
        StateMachine.add('PATH_COMPLETE', PathComplete(),
                           transitions={'success':'GET_PATH'})

    outcome = sm.execute()
