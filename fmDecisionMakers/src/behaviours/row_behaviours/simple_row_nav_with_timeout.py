#!/usr/bin/env python

# Import generic python libraries
import threading

# Import generic ros libraries
import roslib; 
roslib.load_manifest("fmDecisionMakers")
import rospy

import actionlib

import smach
import smach_ros

# Actions used in this statemachine
from fmExecutors.msg import navigate_in_row_simpleAction, navigate_in_row_simpleGoal

from fmMsgs.msg import row
import behaviours

def on_rows(ud,msg):
    if msg.leftvalid and msg.rightvalid:
        return False
    else:
        return True
    
def force_preempt(a):
    return True

def build_row_nav_sm(row_goal,timeout):
    """
        Build the navigate in-row statemachine
        This statemachine only navigates the row and returns succeeded when 
        entering headland
    """
    
    
    find_row_timeout_sm = smach.Concurrence(outcomes=['succeeded','aborted','preempted'], 
                           default_outcome='aborted',
                           outcome_map={
                                        "aborted":{'TIMEOUT':'succeeded','FIND_ROW':'preempted'}, 
                                        "succeeded":{'FIND_ROW':'invalid'}},
                           child_termination_cb=force_preempt)
    
    with find_row_timeout_sm:
        smach.Concurrence.add("FIND_ROW", smach_ros.MonitorState("/fmExtractors/rows", row, on_rows, -1))
        smach.Concurrence.add("TIMEOUT" , behaviours.WaitState(timeout))
    
    
    row_navigator = smach.StateMachine(outcomes=["succeeded","aborted","preempted"])
    
    with row_navigator:
        smach.StateMachine.add("ROW_NAVIGATOR",
                               smach_ros.SimpleActionState("/fmExecutors/navigate_in_row_simple",navigate_in_row_simpleAction, goal=row_goal), 
                               transitions = {"succeeded":"succeeded","aborted":"aborted","preempted":"preempted"}, 
                               )
        
    
    return row_navigator