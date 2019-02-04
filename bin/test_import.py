#! /usr/bin/env python3

import bee_tea.bt_action_node as ban
print('COULD IMPORT BT_ACTION_NODE, SUCCESS')

n = ban.BT_ActionNode(name='test')
n.act(goal='s')



