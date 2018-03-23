#!/usr/bin/env python


import rospy
import smach
import smach_ros

CONSTANT = 'hello'

# define state Foo
class Foo(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1','outcome2'],
                             input_keys=['foo_counter_in', 'foo_list_in'],
                             output_keys=['foo_counter_out', 'foo_list_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing state FOO')
        if userdata.foo_counter_in < 3:
            temp_counter = userdata.foo_counter_in + 1
            userdata.foo_counter_out = temp_counter
            userdata.foo_list_in.append(temp_counter)
            userdata.foo_list_out = userdata.foo_list_in
            global CONSTANT
            rospy.loginfo('FOO: {}'.format(CONSTANT))
            CONSTANT = 'goodbye'
            return 'outcome1'
        else:
            return 'outcome2'


# define state Bar
class Bar(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['outcome1'],
                             input_keys=['bar_counter_in', 'bar_list_in'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
        rospy.loginfo('Counter = %f'%userdata.bar_counter_in)
        rospy.loginfo('List = {}'.format(userdata.bar_list_in))
        rospy.loginfo('BAR: {}'.format(CONSTANT))
        return 'outcome1'


def main():
    global CONSTANT
    CONSTANT = 'erro'

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4'])
    sm.userdata.sm_counter = 0
    sm.userdata.test_list = []

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('FOO', Foo(),
                               transitions={'outcome1':'BAR',
                                            'outcome2':'outcome4'},
                               remapping={'foo_counter_in':'sm_counter',
                                          'foo_counter_out':'sm_counter',
                                          'foo_list_in':'test_list',
                                          'foo_list_out':'test_list'})
        smach.StateMachine.add('BAR', Bar(),
                               transitions={'outcome1':'FOO'},
                               remapping={'bar_counter_in':'sm_counter',
                                          'bar_list_in':'test_list'})


    # Execute SMACH plan
    return sm.execute()
