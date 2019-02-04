# Bee_Tea
A behaviour tree implementation on ROS using Python. Can use ROS actionlib servers as leaf nodes.

See scripts/bt_example.py for an example of how to construct a BT and use a ROS action server at a leaf node.

Terminal 1:
```
$ rosrun bee_tea bt_action_node.py test
```


Terminal 2:
```
$ rosrun bee_tea bt_example.py
```


You should see '... test received goal: s' and then a count to 5 
then '... test completed successfully' in terminal 1.

And something similar to:
```
[INFO] [1537881839.971348]: test is waiting for its action node
[INFO] [1537881840.042342]: test connected to action node
Traversal: [[['c eq 10'], 'c+=1', ['a>b', 'a+=1'], 'inc b'], 'test']
FB:root:RUNNING
   SEQ:seq1:RUNNING
      [!(c eq 10)]:SUCCESS
      [c+=1]:SUCCESS
      FB:fb1:RUNNING
         [a>b]:FAILURE
         [a+=1]:RUNNING
      [inc b]:None
   A:[test]:None

[INFO] [1537881840.664192]: test started
FB:root:RUNNING
   SEQ:seq1:FAILURE
      [!(c eq 10)]:FAILURE
      [c+=1]:None
      FB:fb1:None
         [a>b]:None
         [a+=1]:None
      [inc b]:None
   A:[test]:None -> RUNNING

[INFO] [1537881840.698114]: test received feedback:RUNNING
FB:root:RUNNING
   SEQ:seq1:FAILURE
      [!(c eq 10)]:FAILURE
      [c+=1]:None
      FB:fb1:None
         [a>b]:None
         [a+=1]:None
      [inc b]:None
   A:[test]:RUNNING

[INFO] [1537881846.699710]: test received feedback:SUCCESS
[INFO] [1537881846.763419]: test is done with:SUCCESS
FB:root:SUCCESS
   SEQ:seq1:FAILURE
      [!(c eq 10)]:FAILURE
      [c+=1]:None
      FB:fb1:None
         [a>b]:None
         [a+=1]:None
      [inc b]:None
   A:[test]:SUCCESS -> None
```

in terminal 2


You should implement something that is not counting to 5 in the `act` function.


`_fail`, `_succeed`, `_preempt_cb`, and `_execute_cb`
methods of `BT_ActionNode` can be modified to do specific stuff if need be. 


I suggest superclassing bt_action_node.
Do:


```
from bee_tea.bt_action_node import BT_ActionNode

class My_Action(BT_ActionNode):
    def act(self, goal):
	    # do things with goal
```
