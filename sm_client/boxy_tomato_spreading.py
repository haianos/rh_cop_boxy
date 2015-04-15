#!/usr/bin/env python

## Enea Scioni, Gianni Borghesan
## Boxy Spreading Tomato, Test I - RIGHT ARM ONLY
## Description: Simple motions to spread tomato on pizza
# parameters:
# -- simulation:
#   * True (Interaction constraints disabled, only positioning)
#   * False (Interaction constraints enabled = real demo)
# -- angle
# -- radius


import roslib; roslib.load_manifest('rh_cop')
import rospy
import smach
import smach_ros
from rh_cop_msgs.msg import *
from simple_sm_action import SimpleRHClient, singleEventClient, multipleEventClient, addTransform
from copy import deepcopy
from numpy import pi, cos, sin, sqrt

taskdb = {}

### Settings here!!!!
tframe="rightarm"
simulation=True
radius=0.2;   # [m]
angle= pi/2;  # [rad]
spoon_lenght = 0.23  # [m] from gripper frame to spoon tip
spoon_offset = 0.00  # [m] if spoon not exactly centered in the gripper grasp
top_height = 0.15    # [m] on top of pizza, before to go down 
#### End_settings


##### Some common defs 

point_origin = EntitySpec()
point_origin.type = EntitySpec.POINT;
point_origin.point.x = 0
point_origin.point.y = 0
point_origin.point.z = 0

point_xtrasl = EntitySpec()
point_xtrasl.type = EntitySpec.POINT;
point_xtrasl.point.x = 0.0 
point_xtrasl.point.y = spoon_offset #Our spoon dimension
point_xtrasl.point.z = spoon_lenght #Our spoon dimension

point_ztrasl = EntitySpec()
point_ztrasl.type = EntitySpec.POINT;
point_ztrasl.point.x = 0
point_ztrasl.point.y = 0
point_ztrasl.point.z = top_height

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

znversor = EntitySpec()
znversor.type = EntitySpec.VERSOR;
znversor.versor.x = 0 
znversor.versor.y = 0 
znversor.versor.z = -1 

xversor = EntitySpec()
xversor.type = EntitySpec.VERSOR;
xversor.versor.x = 1
xversor.versor.y = 0 
xversor.versor.z = 0 

yversor = EntitySpec()
yversor.type = EntitySpec.VERSOR;
yversor.versor.x = 0
yversor.versor.y = 1 
yversor.versor.z = 0 

spoon_versor = EntitySpec()
spoon_versor.type = EntitySpec.VERSOR;
spoon_versor.versor.x = 0   # angle spoon
spoon_versor.versor.y = -sqrt(2)/2  # angle spoon
spoon_versor.versor.z = sqrt(2)/2

approaching_traj = TrajectorySpec()
approaching_traj.type = TrajectorySpec.TRAPEZOIDAL_WITH_DURATION
approaching_traj.target   = 0.0
approaching_traj.vel_max  = 0.1
approaching_traj.acc_max  = 0.1
approaching_traj.duration = 5.0


push_force_traj = TrajectorySpec()
push_force_traj.type = TrajectorySpec.CONSTANT
push_force_traj.target   = -2.0

to_radius_traj = deepcopy(approaching_traj)
to_radius_traj.target   = radius

c_elbow_up = ConstraintSpec()
c_elbow_up.constraint_id = "elbow_above_table_constr";
c_elbow_up.behaviour.type = BehaviourSpec.POSITION_LIMIT;
c_elbow_up.behaviour.specification = 1.0;
c_elbow_up.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
c_elbow_up.output_expression.p1.object_frame = "elbow_right";
c_elbow_up.output_expression.p1.entity=point_origin;
c_elbow_up.output_expression.p2.object_frame = "table";
c_elbow_up.output_expression.p2.entity=zline;
c_elbow_up.tr_gen=approaching_traj;
c_elbow_up.tr_gen_upper = 100
c_elbow_up.tr_gen_lower = 0.1


c_spoon_ok= ConstraintSpec()
c_spoon_ok.constraint_id = "c_spoon_ok";
c_spoon_ok.behaviour.type = BehaviourSpec.POSITION_LIMIT;
c_spoon_ok.behaviour.specification = 0.7;
c_spoon_ok.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_VERSORS;
c_spoon_ok.output_expression.p1.object_frame = tframe;
c_spoon_ok.output_expression.p1.entity=spoon_versor;
c_spoon_ok.output_expression.p2.object_frame = "table";
c_spoon_ok.output_expression.p2.entity=znversor;
c_spoon_ok.tr_gen=approaching_traj;
c_spoon_ok.tr_gen_upper = pi/40
c_spoon_ok.tr_gen_lower = -10


## FILL IN moveArmDown
g = TaskGoal()
g.id = "MoveAboveTheCenter"
g.primary.append(c_elbow_up)
g.primary.append(c_spoon_ok)
#tframe="spoon"
c1 = ConstraintSpec()
c1.constraint_id = "hand_above_table_constr";
c1.behaviour.type = BehaviourSpec.POSITIONING;
c1.behaviour.specification = 1.5;
c1.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE;
c1.output_expression.p1.object_frame = tframe;
c1.output_expression.p1.entity=point_xtrasl;
c1.output_expression.p2.object_frame = "table";
c1.output_expression.p2.entity=point_ztrasl;
c1.tr_gen=approaching_traj;
g.primary.append(c1)

m = MonitorSpec()
m.monitor_id = "above_table_mon";
m.monitored_expr = c1.output_expression;
m.event_risen = "e_done";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.015;
m.upper_bound = +0.015;
g.monitors.append(m)

taskdb[g.id] = g

## FILL IN moveArmDown
g = TaskGoal()
g.id = "MoveToTheCenter"
g.primary.append(c_elbow_up)
g.primary.append(c_spoon_ok)
if simulation:
    c_main = ConstraintSpec()
    c_main.constraint_id = "to_table_constr_z";
    c_main.behaviour.type = BehaviourSpec.POSITIONING;
    c_main.behaviour.specification = 1.5;
    c_main.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
    c_main.output_expression.p1.object_frame = tframe;
    c_main.output_expression.p1.entity=point_xtrasl;
    c_main.output_expression.p2.object_frame = "table";
    c_main.output_expression.p2.entity=zline;
    c_main.tr_gen=approaching_traj;
else:
    c_main = ConstraintSpec()
    c_main.constraint_id = "to_table_constr_z_force";
    c_main.behaviour.type = BehaviourSpec.INTERACTION;
    c_main.behaviour.specification = 0.02;
    c_main.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
    c_main.output_expression.p1.object_frame = tframe;
    c_main.output_expression.p1.entity=point_xtrasl;
    c_main.output_expression.p2.object_frame = "table";
    c_main.output_expression.p2.entity=zline;
    c_main.tr_gen=push_force_traj;    


g.primary.append(c_main)


c = ConstraintSpec()
c.constraint_id = "to_table_constr_x";
c.behaviour.type = BehaviourSpec.POSITIONING;
c.behaviour.specification = 1.5;
c.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
c.output_expression.p1.object_frame = tframe;
c.output_expression.p1.entity=point_xtrasl;
c.output_expression.p2.object_frame = "table";
c.output_expression.p2.entity=xline;
c.tr_gen=approaching_traj;
g.primary.append(c)

c = deepcopy(c)
c.constraint_id = "to_table_constr_y";
c.output_expression.p2.entity=yline;
g.primary.append(c)


m = MonitorSpec()
m.monitor_id = "to_table_mon";

m.monitored_expr = c_main.output_expression;
m.event_risen = "e_done";
m.comparison_type = MonitorSpec.MORE;
if simulation:
    m.comparison_type = MonitorSpec.IN_INTERVAL;
    m.monitored_variable_type = MonitorSpec.POS;
    m.lower_bound = -0.015;
    m.upper_bound = +0.015;
else:
    m.comparison_type = MonitorSpec.OUT_INTERVAL;
    m.monitored_variable_type = MonitorSpec.FOR;
    m.upper_bound = 1.5;
    m.lower_bound = -1.5;
    
g.monitors.append(m)

taskdb[g.id] = g


## FILL IN moveTo the first point 

dirline = EntitySpec()
dirline.type = EntitySpec.LINE;
dirline.line.p.x = 0
dirline.line.p.y = 0
dirline.line.p.z = 0
dirline.line.dir.x = cos(angle)
dirline.line.dir.y = sin(angle)
dirline.line.dir.z = 0

perpline = EntitySpec()
perpline.type = EntitySpec.LINE;
perpline.line.p.x = 0
perpline.line.p.y = 0
perpline.line.p.z = 0
perpline.line.dir.x = cos(angle+pi/2)
perpline.line.dir.y = sin(angle+pi/2)
perpline.line.dir.z = 0


g = TaskGoal()
g.id = "MoveToThePeripery1"
g.primary.append(c_elbow_up)
g.primary.append(c_spoon_ok)
if simulation:
    c = ConstraintSpec()
    c.constraint_id = "to_table_constr_z";
    c.behaviour.type = BehaviourSpec.POSITIONING;
    c.behaviour.specification = 1.5; #0.5;
    c.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
    c.output_expression.p1.object_frame = tframe;
    c.output_expression.p1.entity=point_xtrasl;
    c.output_expression.p2.object_frame = "table";
    c.output_expression.p2.entity=zline;
    c.tr_gen=approaching_traj;
else:
    c = ConstraintSpec()
    c.constraint_id = "to_table_constr_z_force";
    c.behaviour.type = BehaviourSpec.INTERACTION;
    c.behaviour.specification = 0.1;
    c.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
    c.output_expression.p1.object_frame = tframe;
    c.output_expression.p1.entity=point_xtrasl;
    c.output_expression.p2.object_frame = "table";
    c.output_expression.p2.entity=zline;
    c.tr_gen=push_force_traj;   
#     c = ConstraintSpec()
#     c.constraint_id = "to_table_constr_z";
#     c.behaviour.type = BehaviourSpec.INTERACTION;
#     c.behaviour.specification = 0.5;
#     c.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
#     c.output_expression.p1.object_frame = tframe;
#     c.output_expression.p1.entity=point_xtrasl;
#     c.output_expression.p2.object_frame = "table";
#     c.output_expression.p2.entity=zline;
#     c.tr_gen=approaching_traj;
#     c.tr_gen.target=5;
#     print("error!! put here force control along z")
g.primary.append(c)

c_main =  ConstraintSpec()
c_main.constraint_id = "to_table_constr_movement_direction"; 
c_main.behaviour.type = BehaviourSpec.POSITIONING;
c_main.behaviour.specification = 1.5; #0.5;
c_main.output_expression.expression = GeometricExpressionSpec.PROJECTION_OF_POINT_ON_LINE;
c_main.output_expression.p1.object_frame = tframe;
c_main.output_expression.p1.entity=point_xtrasl;
c_main.output_expression.p2.object_frame = "table";
c_main.output_expression.p2.entity=dirline;
c_main.tr_gen=to_radius_traj;
 
    
g.primary.append(c_main)

c = deepcopy(c_main)
c.constraint_id = "to_table_constr_perp_movement_direction";
c.output_expression.p2.entity=perpline;
c.tr_gen.target = 0
g.primary.append(c)


m = MonitorSpec()
m.monitor_id = "to_radius_mon";
m.monitored_expr = c_main.output_expression;
m.event_risen = "e_done";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = radius-0.02;
m.upper_bound = radius+0.02;
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
                               transitions={'e_op_done':'MovingAboveTheCenter1','e_op_failed':'aborted'})
      
      smach.StateMachine.add('MovingAboveTheCenter1', singleEventClient(client,"MoveAboveTheCenter",1,'e_done'), 
                               transitions={'e_done':'MoveToTheCenter1','e_aborted':'aborted'})
          
      smach.StateMachine.add('MoveToTheCenter1', singleEventClient(client,"MoveToTheCenter",1,'e_done'), 
                               transitions={'e_done':'MoveToThePeripery1','e_aborted':'aborted'})
      
      smach.StateMachine.add('MoveToThePeripery1', singleEventClient(client,"MoveToThePeripery1",1,'e_done'), 
                            transitions={'e_done':'MovingAboveTheCenter2','e_aborted':'aborted'})
       
      smach.StateMachine.add('MovingAboveTheCenter2', singleEventClient(client,"MoveAboveTheCenter",1,'e_done'), 
                              transitions={'e_done':'MovingAboveTheCenter1','e_aborted':'aborted'})   
                              
      
      smach.StateMachine.add('doNothing', idle(), 
                               transitions={'e_idle':'succeeded'})
                              
    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()