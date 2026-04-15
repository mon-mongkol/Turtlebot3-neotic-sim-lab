#! /bin/bash


source /opt/ros/melodic/setup.sh

source ~/aragorn_ws/devel/setup.bash



rostopic pub -1 /autodocking_server/goal ir_auto_docking/AutodockOperateActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  order: 0" 

sleep 10

rostopic pub -1 /autodocking_server/cancel actionlib_msgs/GoalID "stamp:
  secs: 0
  nsecs: 0
id: ''"




