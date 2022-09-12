#!/usr/bin/python

import moveit_commander
from moveit_commander import MoveGroupCommander
import rospy
import sys
import numpy as np
import pandas as pd
from copy import deepcopy
from scipy.spatial.transform import Rotation as R
            
class JointRecorder(object):
    def __init__(self, filename, rate):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)

        # 初始化ROS节点
        rospy.init_node('moveit_record_dmp', anonymous=True)
                        
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = MoveGroupCommander('plan')
       
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置目标位置所使用的参考坐标系
        arm.set_pose_reference_frame('base_link')
                
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.01)
        arm.set_goal_orientation_tolerance(0.1)

        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
  
        """
        Records joint data to a file at a specified rate.
        """
        self._filename = filename
        self._raw_rate = rate
        self._rate = rospy.Rate(rate)
        self._start_time = rospy.get_time()
        self._done = False
        self.arm=arm
        self.end_effector_link=end_effector_link
    def stop(self):
        """
        Stop recording.
        """
        self._done = True

    def done(self):
        """
        Return whether or not recording is done.
        """
        if rospy.is_shutdown():
            self.stop()
        return self._done

    def set(self):
        #从home到forwad，分别获取初始和末端的姿态
        self.arm.set_named_target('home')
        self.arm.go()
        self.initial_pos=self.arm.get_current_pose(self.end_effector_link)
        self.arm.set_named_target('forward')
        self.arm.go()
        self.goal_pose=self.arm.get_current_pose(self.end_effector_link)
        #获取起点和终点姿态完毕后，将机械臂末端置于一个随机位置，以while循环：防止程序结束。
        temp_pose = self.arm.get_current_pose(self.end_effector_link).pose
        wpose = deepcopy(temp_pose)
        wpose.position.x -= 0.1
        wpose.position.y -= 0.1
        self.arm.set_pose_target(wpose)
        self.arm.go()
        rospy.sleep(2)

    def _time_stamp(self):
        return rospy.get_time() - self._start_time

    
    def record(self):
        pos_record_x = list()
        pos_record_y = list()
        pos_record_z = list()
        orientation_record_roll=list()
        orientation_record_pitch=list()
        orientation_record_yaw=list()
        record_enable = False
        while True:
            print("Waiting <moveit_cartesian_dmp>...")
            # 获取当前位姿数据
            current_pose = self.arm.get_current_pose(self.end_effector_link)
            #位置
            endpoint_pose = [current_pose.pose.position.x,
                            current_pose.pose.position.y,
                            current_pose.pose.position.z]
            #姿态（四元数）
            endpoint_orientation=[current_pose.pose.orientation.x,
                            current_pose.pose.orientation.y,
                            current_pose.pose.orientation.z,
                            current_pose.pose.orientation.w]
          
            if (record_enable == False) and (np.sqrt(
                (endpoint_pose[0] - self.initial_pos.pose.position.x) ** 2 + (endpoint_pose[1] - self.initial_pos.pose.position.y) ** 2 + (
                        endpoint_pose[2] - self.initial_pos.pose.position.z** 2)+(endpoint_orientation[0]-self.initial_pos.pose.orientation.x)**2+
                         (endpoint_orientation[1]-self.initial_pos.pose.orientation.y)**2+(endpoint_orientation[2]-self.initial_pos.pose.orientation.z)**2
                         +(endpoint_orientation[3]-self.initial_pos.pose.orientation.w)**2< 0.005)):
                record_enable = True
            if (np.sqrt((endpoint_pose[0] - self.goal_pose.pose.position.x) ** 2 + (endpoint_pose[1] - self.goal_pose.pose.position.y) ** 2 + (
                    endpoint_pose[2] - self.goal_pose.pose.position.z) ** 2 +(endpoint_orientation[0]-self.goal_pose.pose.orientation.x)**2
                    +(endpoint_orientation[1]-self.goal_pose.pose.orientation.y)**2+(endpoint_orientation[2]-self.goal_pose.pose.orientation.z)**2
                    +(endpoint_orientation[3]-self.goal_pose.pose.orientation.w)**2)< 0.005):

                record_enable = False
                break
            
            if record_enable == True:
                r = R.from_quat([endpoint_orientation[0],endpoint_orientation[1],endpoint_orientation[2],endpoint_orientation[3]])
                endpoint_orientation_eul=r.as_euler(seq='zyz',degrees=False)
                pos_record_x.append(endpoint_pose[0])
                pos_record_y.append(endpoint_pose[1])
                pos_record_z.append(endpoint_pose[2])
                orientation_record_roll.append(endpoint_orientation_eul[0])
                orientation_record_pitch.append(endpoint_orientation_eul[1])
                orientation_record_yaw.append(endpoint_orientation_eul[2])
            
            self._rate.sleep()
        data = np.vstack((pos_record_x, pos_record_y, pos_record_z,orientation_record_roll,orientation_record_pitch,orientation_record_yaw))
        df = pd.DataFrame(data)
        df.to_csv(self._filename , index=False, header=None)


def main():
    #开始时让机械臂移动到随机位置，然后让机械臂
    print("Initializing node... ")
    # 初始化move_group的API
    moveit_commander.roscpp_initialize(sys.argv)

    # 初始化ROS节点
    rospy.init_node('moveit_record_dmp', anonymous=True)
                    
    # 初始化需要使用move group控制的机械臂中的arm group
    arm = MoveGroupCommander('plan')
    
    # 当运动规划失败后，允许重新规划
    arm.allow_replanning(True)
    
    # 设置目标位置所使用的参考坐标系
    arm.set_pose_reference_frame('base_link')
            
    # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.1)

    Recorder = JointRecorder('demo_trajectory_for_discrete_dmps.csv', 100)
    print("Recording...")
    Recorder.set()#获取起始点和终点的姿态信息
    Recorder.record() 
    print("\nRecording Done.")


if __name__ == '__main__':
    main()
