#ROS2 related
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from ur_action_interfaces.action import Mapping
from ur_action_interfaces.action import Scooping

#State machine
from transitions import Machine, State
from transitions.extensions import GraphMachine

class StateMachineNode(Node):
    def __init__(self):        
        #Create ROS node
        super().__init__("state_machine")
        self.get_logger().info("State machine node is up")
        #Create action clients
        self._action_client_mapping = ActionClient(self, Mapping, 'map_surroundings_server')
        self._action_client_scooping = ActionClient(self, Scooping, 'scooping_server')

        #Environment
        self.pt = [0,0,0]
        self.fail_reason = "NA"
        self.end = False
        
    def mapping(self):
        val = input("Press enter to begin")
        goal = Mapping.Goal()
        goal.begin = 1;
        self.get_logger().info("Waiting for mapping server...")
        self._action_client_mapping.wait_for_server()
        self._send_goal_future = self._action_client_mapping.send_goal_async(goal)
        self.get_logger().info("Sent goal to mapping server")       
        self._send_goal_future.add_done_callback(self.mapping_response_callback)
        
    def mapping_response_callback(self, future):
        goal_handle = future.result()
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.mapping_result_callback)        
    
    def mapping_result_callback(self, future):
        goal_handle = future.result()
        outcome = future.result().result.outcome
        if (outcome == "Completed"):
            self.mapping_success()
        else:
            pass
    
    def wait_for_pts(self):
        self.get_logger().info("Current state: "+ self.state)
        self.get_logger().info("Waiting for points...")
        valid = False
        while (not valid):
            if (len(self.pt) != 3 or (self.pt[0] == 0 and self.pt[1] == 0 and self.pt[2] == 0)):
                try:                        
                    self.pt[0], self.pt[1], self.pt[2] = map(float,input("Please enter three non-zero numbers (x,y,z): ").split(','))
                except TypeError:
                    self.get_logger().info("Invalid type, try again.")
                except ValueError:
                    if (self.pt[0] == "0"):
                        self.fail_reason = "Aborted wait_for_pts"
                        self.abort()
                    self.get_logger().info("Incorrect number of values, try again.")
                else:
                    valid = True
        self.get_logger().info("Sending point: (%f, %f, %f)"%(self.pt[0], self.pt[1], self.pt[2]))
        self.pts_obtained()
        
    def scooping(self):
        self.get_logger().info("Current state: "+ self.state)
        goal = Scooping.Goal()
        goal.goal = [self.pt[0],self.pt[1],self.pt[2]];
        self.get_logger().info("Waiting for scooping server...")
        self._action_client_scooping.wait_for_server()
        self._send_goal_future = self._action_client_scooping.send_goal_async(goal)
        self.get_logger().info("Sent goal to scooping server")       
        self._send_goal_future.add_done_callback(self.scooping_response_callback)
        
    def scooping_response_callback(self, future):
        goal_handle = future.result()
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.scooping_result_callback)        
    
    def scooping_result_callback(self, future):
        goal_handle = future.result()
        outcome = future.result().result.outcome
        if (outcome == "Completed"):
            self.scooping_success()
        else:
            pass
            
    def fail_st(self):
        self.get_logger().info("Reason for entering fail state: " + self.fail_reason)
        self.end = True
        return
        
def run_sm(node):
    #Create state machines
    states = [State(name='mapping',on_enter=['mapping']),
              State(name='wait_for_pts',on_enter=['wait_for_pts']),
              State(name='scooping', on_enter=['scooping']),
              'tool_changing','collecting','end',
              State(name='fail_st', on_enter=['fail_st'])]
    machine = GraphMachine(model=node, states=states, initial='mapping' )
    machine.add_transition('mapping_success', 'mapping', 'wait_for_pts')
    machine.add_transition('mapping_fail', 'mapping', 'fail_st')
    machine.add_transition('pts_obtained', 'wait_for_pts', 'scooping')
    machine.add_transition('abort', 'wait_for_pts', 'fail_st')
    machine.add_transition('scooping_success', 'scooping', 'collecting')
    machine.add_transition('scooping_fail', 'scooping', 'fail_st')
    machine.add_transition('hard', 'scooping', 'tool_changing')
    machine.add_transition('tool_change_success', 'tool_changing', 'scooping')
    machine.add_transition('tool_change_fail', 'tool_changing', 'fail_st')
    machine.add_transition('collecting_success', 'collecting', 'end')
    machine.add_transition('collecting_fail', 'collecting', 'fail_st')
    if (node.end == True):
        return
    
    node.get_logger().info("State machine has been created")
    node.get_graph().draw('my_state_diagram.png', prog='dot')
    node.get_logger().info("Current state: " + node.state)
    #node.to_wait_for_pts()
    node.mapping()
   
def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    run_sm(node)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
