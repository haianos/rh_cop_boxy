#!/usr/bin/env python

## Enea Scioni, Gianni Borghesan
## Boxy Test - RIGHT ARM ONLY
## Description: Moving the end effector along Axis of TABLE fixed frame

import roslib; roslib.load_manifest('rh_cop')
import rospy
import smach
import smach_ros
from rh_cop_msgs.msg import *
from simple_sm_action import SimpleRHClient, singleEventClient, multipleEventClient, addTransform

##### Some common defs 

point_origin = EntitySpec()
point_origin.type = EntitySpec.POINT;
point_origin.point.x = 0
point_origin.point.y = 0
point_origin.point.z = 0

xline = EntitySpec()
xline.type = EntitySpec.LINE;
xline.line.p.x = 0
xline.line.p.y = 0
xline.line.p.z = 0
xline.line.dir.x = 1 
xline.line.dir.y = 0
xline.line.dir.z = 0

yline = EntitySpec()
yline.type = EntitySpec.LINE;
yline.line.p.x = 0
yline.line.p.y = 0
yline.line.p.z = 0
yline.line.dir.x = 0 
yline.line.dir.y = 1
yline.line.dir.z = 0

zline = EntitySpec()
zline.type = EntitySpec.LINE;
zline.line.p.x = 0
zline.line.p.y = 0
zline.line.p.z = 0
zline.line.dir.x = 0 
zline.line.dir.y = 0
zline.line.dir.z = 1

approaching_traj = TrajectorySpec()
approaching_traj.type = TrajectorySpec.TRAPEZOIDAL_WITH_DURATION
approaching_traj.target   = 0.0
approaching_traj.vel_max  = 0.1
approaching_traj.acc_max  = 0.1
approaching_traj.duration = 6.0

away_traj = TrajectorySpec()
away_traj.type = TrajectorySpec.TRAPEZOIDAL_WITH_DURATION
away_traj.target   = 0.25
away_traj.vel_max  = 0.1
away_traj.acc_max  = 0.1
away_traj.duration = 4.0

position_tol = 0.005
#### End entities defs

######### Define TASK
taskdb = {}

px_constraint = ConstraintSpec()
px_constraint.constraint_id = "px_constraint";
px_constraint.behaviour.type = BehaviourSpec.POSITIONING;
px_constraint.behaviour.specification = 0.1;
px_constraint.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
px_constraint.output_expression.p1.object_frame = "rightarm";
px_constraint.output_expression.p1.entity = point_origin;
px_constraint.output_expression.p2.object_frame = "table";
px_constraint.output_expression.p2.entity = xline;
px_constraint.tr_gen = approaching_traj;

py_constraint = ConstraintSpec()
py_constraint.constraint_id = "py_constraint";
py_constraint.behaviour.type = BehaviourSpec.POSITIONING;
py_constraint.behaviour.specification = 0.1;
py_constraint.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
py_constraint.output_expression.p1.object_frame = "rightarm";
py_constraint.output_expression.p1.entity = point_origin;
py_constraint.output_expression.p2.object_frame = "table";
py_constraint.output_expression.p2.entity = yline;
py_constraint.tr_gen = approaching_traj;

pz_constraint = ConstraintSpec()
pz_constraint.constraint_id = "pz_constraint";
pz_constraint.behaviour.type = BehaviourSpec.POSITIONING;
pz_constraint.behaviour.specification = 0.1;
pz_constraint.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
pz_constraint.output_expression.p1.object_frame = "rightarm";
pz_constraint.output_expression.p1.entity = point_origin;
pz_constraint.output_expression.p2.object_frame = "table";
pz_constraint.output_expression.p2.entity = zline;
pz_constraint.tr_gen = approaching_traj;

pz_away_constraint = ConstraintSpec()
pz_away_constraint.constraint_id = "pz_away_constraint";
pz_away_constraint.behaviour.type = BehaviourSpec.POSITIONING;
pz_away_constraint.behaviour.specification = 0.1;
pz_away_constraint.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
pz_away_constraint.output_expression.p1.object_frame = "rightarm";
pz_away_constraint.output_expression.p1.entity = point_origin;
pz_away_constraint.output_expression.p2.object_frame = "table";
pz_away_constraint.output_expression.p2.entity = zline;
pz_away_constraint.tr_gen = away_traj;

m = MonitorSpec()
m.monitor_id = "arm_down";
m.monitored_expr = pz_constraint.output_expression;
m.event_risen = "e_arm_down";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -position_tol;
m.upper_bound = +position_tol;

## FILL IN moveArmDown
g = TaskGoal()
g.id = "moveArmDown"
g.primary.append(px_constraint)
g.primary.append(py_constraint)
g.primary.append(pz_constraint)
g.monitors.append(m)
taskdb[g.id] = g

# FILL IN moveArmUp
g = TaskGoal()
g.id = "moveArmUp"

g.primary.append(px_constraint)
g.primary.append(py_constraint)
g.primary.append(pz_away_constraint)
m = MonitorSpec()
m.monitor_id = "arm_up";
m.monitored_expr = pz_away_constraint.output_expression;
m.event_risen = "e_arm_up";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -position_tol+away_traj.target;
m.upper_bound = +position_tol+away_traj.target;
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
    sm = smach.StateMachine(outcomes=['succeeded','aborted'])
    
    # smach introspection FSM
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()

    # open actionlib client
    client = SimpleRHClient('cop_interface',taskdb)
    
    # Open the container
    with sm:        
      # Add states to the container
      smach.StateMachine.add('loadTableFrame', addTransform('add','table','triangle_right_arm_link'), 
                               transitions={'e_op_done':'movingArmDown1','e_op_failed':'aborted'})
      smach.StateMachine.add('movingArmDown1', singleEventClient(client,"moveArmDown",1,'e_arm_down'), 
                               transitions={'e_arm_down':'movingArmUp1','e_aborted':'aborted'})
      smach.StateMachine.add('movingArmUp1', singleEventClient(client,"moveArmUp",1,'e_arm_up'), 
                               transitions={'e_arm_up':'movingArmDown2','e_aborted':'aborted'})
      smach.StateMachine.add('movingArmDown2', singleEventClient(client,"moveArmDown",1,'e_arm_down'), 
                               transitions={'e_arm_down':'movingArmUp2','e_aborted':'aborted'})
      smach.StateMachine.add('movingArmUp2', singleEventClient(client,"moveArmUp",1,'e_arm_up'), 
                               transitions={'e_arm_up':'doNothing','e_aborted':'aborted'}) 
      smach.StateMachine.add('doNothing', idle(), 
                               transitions={'e_idle':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
