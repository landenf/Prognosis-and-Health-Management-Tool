#!/usr/bin/python3
import rospy
from std_msgs.msg import String
import sys
import threading
import rosnode
from collections import OrderedDict
import requests


class NodeMonitor:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        self.node_name = node_name
        self.alive_nodes = set()
        self.lock = threading.Lock()
        self.nodes_to_check = ['node1', 'node2', 'node3']
        self.ConsensusMatrix = {node: {inner_node: None for inner_node in self.nodes_to_check} for node in self.nodes_to_check}
        
        self.pub = rospy.Publisher('/alive_nodes', String, queue_size=10)
        
        rospy.Subscriber('/alive_nodes', String, self.alive_nodes_callback)
         
        rospy.Timer(rospy.Duration(2), self.publish_alive_nodes) #publish every 2
        rospy.Timer(rospy.Duration(5), self.analyze_consensus_matrix) #publish ever 5

    def publish_alive_nodes(self, event):
        alive_nodes_list = self.get_alive_nodes() #save?
        alive_nodes_str = ','.join(alive_nodes_list)
        #rospy.loginfo(f"{self.node_name} publishing alive nodes: {alive_nodes_list}")
        self.pub.publish(alive_nodes_str)

    def get_alive_nodes(self):
        alive = []
        node_names_ros = rosnode.get_node_names()
        node_names = [node.lstrip('/') for node in node_names_ros]
        for node in self.nodes_to_check:
            if node in node_names:
                rospy.loginfo(f"Node {node} is up and running.")
                alive.insert(0, node)
            else:
                rospy.logwarn(f"Node {node} is not running.")
        if self.node_name in alive and alive[0] != self.node_name:
            alive.remove(self.node_name)
            alive.insert(0, self.node_name)
        return alive
    
    def alive_nodes_callback(self, msg):
        with self.lock:
            recieved_alive_nodes = list(OrderedDict.fromkeys(msg.data.split(',')))
            if recieved_alive_nodes:
                sender = list(recieved_alive_nodes)[0]
                if sender in self.ConsensusMatrix:
                    for node in self.nodes_to_check:
                        self.ConsensusMatrix[sender][node] = 1 if node in recieved_alive_nodes else 0
            
            #rospy.loginfo(f"{self.node_name} received alive nodes: {recieved_alive_nodes}")
            #rospy.loginfo(f"Updated ConsensusMatrix for{self.node_name} from {sender}: {self.ConsensusMatrix}")

    def analyze_consensus_matrix(self, event):
        consensus_alive_nodes = []
        num_nodes = len(self.ConsensusMatrix)
        for node in self.nodes_to_check:
            alive_count = 0
            reported_count = 0
            for reporter, reports in self.ConsensusMatrix.items():
                if reports[node] is not None:  
                    reported_count += 1
                    if reports[node] == 1:  
                        alive_count += 1
            if reported_count > num_nodes / 2:
                if alive_count > reported_count / 2:
                    consensus_alive_nodes.append(node)

        rospy.loginfo(f"Consensus alive nodes from {self.node_name}: {consensus_alive_nodes}")
        #print ok or error if missing
        #Call out to server
        nodes_to_send = '-'.join(consensus_alive_nodes)
        url = f"http://127.0.0.1:8080/ROS_Agents/{nodes_to_send}"
        #requests.get(url)
        print(url)
        return consensus_alive_nodes

if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(1)
    node_name = sys.argv[1]
    node_monitor = NodeMonitor(node_name)
    rospy.spin()
