#!/usr/bin/env python

# Enea Scioni, 2015,
# University of Leuven, Belgium

# RoboHow.Cog
# Test FSM Client Example on Boxy Robot, only ARM-RIGHT is used

import roslib; roslib.load_manifest('rh_cop')
import rospy
import smach
import smach_ros
from rh_cop_msgs.msg import *
from simple_sm_action import *

######### Define TASK
taskdb = {}

# Align Constraint Definition
align_table_point = ConstraintSpec()
align_table_point.constraint_id = "align_table_point";
align_table_point.behaviour.type = BehaviourSpec.POSITIONING;
align_table_point.behaviour.specification = 1;
align_table_point.output_expression.expression = GeometricExpressionSpec.LINE_POINT_DISTANCE;
align_table_point.output_expression.p1.object_frame = "table";
align_table_point.output_expression.p1.entity.type = EntitySpec.POINT;
align_table_point.output_expression.p1.entity.point.x = 0;
align_table_point.output_expression.p1.entity.point.y = 0;
align_table_point.output_expression.p1.entity.point.z = 0;
align_table_point.output_expression.p2.object_frame = "rightarm";
align_table_point.output_expression.p2.entity.type = EntitySpec.LINE;
align_table_point.output_expression.p2.entity.line.p.x = 0;
align_table_point.output_expression.p2.entity.line.p.y = 0;
align_table_point.output_expression.p2.entity.line.p.z = 0;
align_table_point.output_expression.p2.entity.line.dir.x = 0;
align_table_point.output_expression.p2.entity.line.dir.y = 0;
align_table_point.output_expression.p2.entity.line.dir.z = 1;
align_table_point.tr_gen.type = TrajectorySpec.TRAPEZOIDAL_WITH_DURATION;
align_table_point.tr_gen.target = 0.0;
align_table_point.tr_gen.vel_max = 0.5;
align_table_point.tr_gen.acc_max = 0.3;
align_table_point.tr_gen.duration = 10.0;


## FILL IN moveArmDown
g = TaskGoal()
g.id = "moveArmClose"

c = ConstraintSpec()
c.constraint_id = "to_table";
c.behaviour.type = BehaviourSpec.POSITIONING;
c.behaviour.specification = 2;
c.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE;
c.output_expression.p1.object_frame = "rightarm";
c.output_expression.p1.entity.type = EntitySpec.POINT;
c.output_expression.p2.entity.point.x = 0;
c.output_expression.p2.entity.point.y = 0;
c.output_expression.p2.entity.point.z = 0;
c.output_expression.p2.object_frame = "table";
c.output_expression.p2.entity.type = EntitySpec.POINT;
c.output_expression.p2.entity.point.x = 0;
c.output_expression.p2.entity.point.y = 0;
c.output_expression.p2.entity.point.z = 0;
c.tr_gen.type = TrajectorySpec.TRAPEZOIDAL_WITH_DURATION;
c.tr_gen.target = 0.0;
c.tr_gen.vel_max = 0.5;
c.tr_gen.acc_max = 0.3;
c.tr_gen.duration = 10.0;
g.primary.append(c)

#g.primary.append(align_table_point)

m = MonitorSpec()
m.monitor_id = "arm_close";
m.monitored_expr = c.output_expression;
m.event_risen = "e_arm_close";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.01;
m.upper_bound = 0.01;

g.monitors.append(m)
taskdb[g.id] = g

# FILL IN moveArmUp
g = TaskGoal()
g.id = "moveArmAway"

c = ConstraintSpec()
c.constraint_id = "away_from_table";
c.behaviour.type = BehaviourSpec.POSITIONING;
c.behaviour.specification = 2;
c.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE;
c.output_expression.p1.object_frame = "rightarm";
c.output_expression.p1.entity.type = EntitySpec.POINT;
c.output_expression.p1.entity.point.x = 0;
c.output_expression.p1.entity.point.y = 0;
c.output_expression.p1.entity.point.z = 0;
  
c.output_expression.p2.object_frame = "table";
c.output_expression.p2.entity.type = EntitySpec.POINT;
c.output_expression.p1.entity.point.x = 0;
c.output_expression.p1.entity.point.y = 0;
c.output_expression.p1.entity.point.z = 0;

c.tr_gen.type = TrajectorySpec.TRAPEZOIDAL;
c.tr_gen.target = 0.3;
c.tr_gen.vel_max = 0.05;
c.tr_gen.acc_max = 0.01;
c.tr_gen.duration = 5.0;
g.primary.append(c)

#g.primary.append(align_table_point)

m = MonitorSpec()
m.monitor_id = "arm_away";
m.monitored_expr = c.output_expression;
m.event_risen = "e_arm_away";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.005+0.3;
m.upper_bound = 0.005+0.3;
g.monitors.append(m)

taskdb[g.id] = g
######### End Define TASK
        
# define state idle
class idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['e_idle'])

    def execute(self, userdata):
        rospy.loginfo('Executing: Idle Task')
        return 'e_idle'
    
def main():
    rospy.init_node('test_client')
    
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])
    
    # smach introspection FSM
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # open actiionlib client
    client = SimpleRHClient('cop_interface',taskdb)
    
    # Open the container
    with sm:
      # Add states to the container
      smach.StateMachine.add('addExternalFrame', addTransform('add','table','triangle_right_arm_link'), 
                               transitions={'e_op_done':'movingArmClose','e_op_failed':'aborted'})
      smach.StateMachine.add('movingArmClose', singleEventClient(client,"moveArmClose",1,'e_arm_close'), 
                               transitions={'e_arm_close':'movingArmAway','e_aborted':'aborted'})
      smach.StateMachine.add('movingArmAway', singleEventClient(client,"moveArmAway",1,'e_arm_away'), 
                               transitions={'e_arm_away':'movingArmClose2','e_aborted':'aborted'})
      smach.StateMachine.add('movingArmClose2', singleEventClient(client,"moveArmClose",1,'e_arm_close'), 
                               transitions={'e_arm_close':'movingArmAway2','e_aborted':'aborted'})
      smach.StateMachine.add('movingArmAway2', singleEventClient(client,"moveArmAway",1,'e_arm_away'), 
                               transitions={'e_arm_away':'doNothing','e_aborted':'aborted'})
      smach.StateMachine.add('doNothing', idle(), 
                               transitions={'e_idle':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()