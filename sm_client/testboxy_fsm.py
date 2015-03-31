#!/usr/bin/env python

import roslib; roslib.load_manifest('rh_cop')
import rospy
import smach
import smach_ros
#from smach_ros import SimpleActionState
#from simple_actionlib_state import SimpleActionState
from rh_cop_msgs.msg import * #TaskAction, TaskGoal, MonitorSpec, ConstraintSpec
#from simple_rh_client import SimpleRHClient
from simple_sm_action import *

######### Define TASK
taskdb = {}
#'lookAt':TaskGoal(),
#              'prepareArm':TaskGoal(),'moveLeftArm':TaskGoal()}

## FILL IN lookAt
#g = taskdb['lookAt']
g = TaskGoal()
g.id = "lookAt"

c = ConstraintSpec()
c.constraint_id = "camera_look_rightarm";
c.behaviour.type = BehaviourSpec.POSITIONING;
c.behaviour.specification = 2;
c.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_LINE_AND_POINT;
c.output_expression.p1.object_frame = "camera";
c.output_expression.p1.entity.type = EntitySpec.LINE;
c.output_expression.p1.entity.line.p.x = 0;
c.output_expression.p1.entity.line.p.y = 0;
c.output_expression.p1.entity.line.p.z = 0;
c.output_expression.p1.entity.line.dir.x = 0;
c.output_expression.p1.entity.line.dir.y = 0;
c.output_expression.p1.entity.line.dir.z = -1;
c.output_expression.p2.object_frame = "rightarm";
c.output_expression.p2.entity.type = EntitySpec.POINT;
c.output_expression.p2.entity.point.x = 0;
c.output_expression.p2.entity.point.y = 0;
c.output_expression.p2.entity.point.z = 0;
c.tr_gen.type = TrajectorySpec.TRAPEZOIDAL;
c.tr_gen.target = 0.0;
c.tr_gen.vel_max = 0.5;
c.tr_gen.acc_max = 0.3;
c.tr_gen.duration = 4.0;
g.primary.append(c)

m = MonitorSpec()
m.monitor_id = "lookAtDone";
m.monitored_expr = c.output_expression;
m.event_risen = "e_infield";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.01; #+tgt
m.upper_bound = 0.01;  #+tgt
g.monitors.append(m)
taskdb['lookAt'] = g

## FILL IN prepareArm
g = TaskGoal()
g.id = "prepareArm"

c = ConstraintSpec()
c.constraint_id = "align_ree_table";
c.behaviour.type = BehaviourSpec.POSITIONING;
c.behaviour.specification = 2;
c.output_expression.expression = GeometricExpressionSpec.ANGLE_BTW_LINE_AND_POINT;
c.output_expression.p1.object_frame = "rightarm";
c.output_expression.p1.entity.type = EntitySpec.LINE;
c.output_expression.p1.entity.line.p.x = 0;
c.output_expression.p1.entity.line.p.y = 0;
c.output_expression.p1.entity.line.p.z = 0;
c.output_expression.p1.entity.line.dir.x = 0;
c.output_expression.p1.entity.line.dir.y = 0;
c.output_expression.p1.entity.line.dir.z = 1;
c.output_expression.p2.object_frame = "table";
c.output_expression.p2.entity.type = EntitySpec.POINT;
c.output_expression.p2.entity.point.x = 0;
c.output_expression.p2.entity.point.y = 0;
c.output_expression.p2.entity.point.z = 0;
c.tr_gen.type = TrajectorySpec.TRAPEZOIDAL;
c.tr_gen.target = 0.0;
c.tr_gen.vel_max = 0.5;
c.tr_gen.acc_max = 0.3;
c.tr_gen.duration = 5.0;
g.primary.append(c)

m = MonitorSpec()
m.monitor_id = "prepareArmDone";
m.monitored_expr = c.output_expression;
m.event_risen = "e_prepared";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.05;
m.upper_bound = 0.05;

g.monitors.append(m)
taskdb['prepareArm'] = g

# FILL IN moveLeftArm
g = TaskGoal()
g.id = "moveLeftArm"

c = ConstraintSpec()
c.constraint_id = "move_left_arm";
c.behaviour.type = BehaviourSpec.POSITIONING;
c.behaviour.specification = 2;
c.output_expression.expression = GeometricExpressionSpec.POINT_TO_POINT_DISTANCE;
c.output_expression.p1.object_frame = "leftarm";
c.output_expression.p1.entity.type = EntitySpec.POINT;
c.output_expression.p1.entity.point.x = 0;
c.output_expression.p1.entity.point.y = 0;
c.output_expression.p1.entity.point.z = 0;
  
c.output_expression.p2.object_frame = "table";
c.output_expression.p2.entity.type = EntitySpec.POINT;
c.output_expression.p2.entity.point.x = 0;
c.output_expression.p2.entity.point.y = 0;
c.output_expression.p2.entity.point.z = 0;

c.tr_gen.type = TrajectorySpec.TRAPEZOIDAL;
c.tr_gen.target = 0.0;
c.tr_gen.vel_max = 0.5;
c.tr_gen.acc_max = 0.3;
c.tr_gen.duration = 2.0;
g.primary.append(c)

m = MonitorSpec()
m.monitor_id = "moveLeftArmDone";
m.monitored_expr = c.output_expression;
m.event_risen = "e_leftarm_moved";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.01;
m.upper_bound = 0.01;
g.monitors.append(m) 

taskdb['moveLeftArm'] = g

# FILL IN moveRightArm
g = TaskGoal()
g.id = "moveRightArm"

c_left_store = c
c = ConstraintSpec()
c.constraint_id = "move_right_arm";
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
c.output_expression.p2.entity.point.x = 0;
c.output_expression.p2.entity.point.y = 0.0;
c.output_expression.p2.entity.point.z = 0.5;
  
c.tr_gen.type = TrajectorySpec.TRAPEZOIDAL;
c.tr_gen.target = 0.0;
c.tr_gen.vel_max = 0.5;
c.tr_gen.acc_max = 0.3;
c.tr_gen.duration = 2.0;
g.primary.append(c_left_store)
g.primary.append(c)

m = MonitorSpec()
m.monitor_id = "moveRightArmDone";
m.monitored_expr = c.output_expression;
m.event_risen = "e_rightarm_moved";
m.comparison_type = MonitorSpec.IN_INTERVAL;
m.monitored_variable_type = MonitorSpec.POS;
m.lower_bound = -0.01;
m.upper_bound = 0.01;
g.monitors.append(m)

taskdb['moveRightArm'] = g

######### End Define TASK

# define state lookAt
class lookAt(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['e_infield'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo('Executing: lookAt Task')
        if self.counter < 3:
            self.counter += 1
            return 'e_infield'


# define state prepareArm
class prepareArm(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['e_prepared'])

    def execute(self, userdata):
        rospy.loginfo('Executing: prepareArm Task')
        return 'e_prepared'
        
# define state idle
class idle(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['e_idle'])

    def execute(self, userdata):
        rospy.loginfo('Executing: Idle Task')
        return 'e_idle'

def monitor_policy(fb,event_name):
    exit_cond = False
    while ( (not rospy.is_shutdown()) and (not exit_cond)):
      print "zzZZ"
      print "end cond "+str(exit_cond)
      rospy.sleep(1)
      print "wake up"
      for ev in fb.events:
        print "exec: "+ev
        if ev=="e_prepared":
          print "FIND"
          exit_cond = True
          break
          
# class wrap container for a state, a lot of boilcode
class testClient(smach.State):
  def __init__(self,client,taskname):
    smach.State.__init__(self, outcomes=['e_infield'])
    self._client = client
    self._taskname = taskname
    self._fb = TaskFeedback()
    
  def execute(self, userdata):
    print self._taskname
    self._client.sendTask(self._taskname,self.monitor_handle)
    exit_cond = False
    while ( (not rospy.is_shutdown()) and (not exit_cond)):
      rospy.sleep(1)
      for ev in self._fb.events:
        print "exec: "+ev
        if ev=="e_infield":
          exit_cond = True
          break
      
    # Cancel the goal
    self._client.deleteTask()
    return 'e_infield'
  
  def monitor_handle(self,fb):
    self._fb = fb

class testClient2(smach.State):
  def __init__(self,client,taskname):
    smach.State.__init__(self, outcomes=['e_prepared'])
    self._client = client
    self._taskname = taskname
    self._fb = TaskFeedback()
    
  def execute(self, userdata):
    print self._taskname
    self._client.sendTask(self._taskname,self.monitor_handle)
    exit_cond = False
    while ( (not rospy.is_shutdown()) and (not exit_cond)):
      rospy.sleep(1)
      for ev in self._fb.events:
        print "exec: "+ev
        if ev=="e_prepared":
          exit_cond = True
          break
      
    # Cancel the goal
    self._client.deleteTask()  
    return 'e_prepared'
  
  def monitor_handle(self,fb):
    self._fb = fb
    
#class singleEventClient(smach.State):
  #def __init__(self,client,taskname,sleeptime,event):
    #smach.State.__init__(self, outcomes=[event])
    #self._event = event
    #self._sleeptime = sleeptime
    #self._client = client
    #self._taskname = taskname
    #self._fb = TaskFeedback()
    
  #def execute(self, userdata):
    #print self._taskname
    #self._client.sendTask(self._taskname,self.monitor_handle)
    #exit_cond = False
    #while ( (not rospy.is_shutdown()) and (not exit_cond)):
      #rospy.sleep(self._sleeptime)
      #for ev in self._fb.events:
        #if ev==self._event:
          #exit_cond = True
          #break
      
    ## Cancel the goal
    #self._client.deleteTask()  
    #return self._event
  
  #def monitor_handle(self,fb):
    #self._fb = fb
    
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
      def gripper_result_cb(userdata, status, result):
        print "lala"
        print len(result.events)
        
      # Add states to the container
      smach.StateMachine.add('addExternalFrame', addTransform('add','table','base_footprint'), 
                               transitions={'e_op_done':'lookingAt','e_op_failed':'aborted'})
      #smach.StateMachine.add('lookingAt',
                              #SimpleActionState('cop_interface',
                                        #TaskAction,
                                        #goal=taskdb['lookAt'],
                                        #result_cb=gripper_result_cb),
                                #transitions={'succeeded':'preparingArm'})
      #smach.StateMachine.add('lookingAt', testClient(client,"lookAt"), 
                               #transitions={'e_infield':'preparingArm'})
      smach.StateMachine.add('lookingAt', singleEventClient(client,"lookAt",1,'e_infield'), 
                               transitions={'e_infield':'preparingArm','e_aborted':'aborted'})
      smach.StateMachine.add('preparingArm', singleEventClient(client,"prepareArm",1,'e_prepared'), 
                               transitions={'e_prepared':'movingLeftArm','e_aborted':'aborted'})
      smach.StateMachine.add('movingLeftArm', singleEventClient(client,"moveLeftArm",1,'e_leftarm_moved'), 
                               transitions={'e_leftarm_moved':'movingRightArm','e_aborted':'aborted'})
      smach.StateMachine.add('movingRightArm', singleEventClient(client,"moveRightArm",1,'e_rightarm_moved'), 
                               transitions={'e_rightarm_moved':'doNothing','e_aborted':'aborted'})
      #smach.StateMachine.add('preparingArm', testClient2(client,"prepareArm"), 
                               #transitions={'e_prepared':'waiting'})
      #smach.StateMachine.add('lookingAt', lookAt(), 
                               #transitions={'e_infield':'preparingArm'})
      #smach.StateMachine.add('preparingArm', prepareArm(), 
                               #transitions={'e_prepared':'waiting'})
      smach.StateMachine.add('doNothing', idle(), 
                               transitions={'e_idle':'succeeded'})

    # Execute SMACH plan
    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()