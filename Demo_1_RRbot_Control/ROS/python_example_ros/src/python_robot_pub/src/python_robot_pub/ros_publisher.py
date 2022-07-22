import rospy
import std_msgs.msg as message

class ROSPublisher():
    """
    Class that publishes data to specified ROS topics
    """
    def __init__(self, node_name:str,topic:str, data:str, queue:int, anonymous=True, signal=False, verbose=True):
        """Initialized a ROS Publisher node.

        Args:
            node_name (str): name to be assigned to the node
            topic (str): topic to publish to
            data (str): data type from the std_msgs library, passed as a string (example: "Float32" -> std_msgs.msg.Float32)
            queue (int): queue size for the publisher
            anonymous (bool, optional): If set to true give unique name to same named nodes. Defaults to True.
            signal (bool, optional): If set to true, allow the use of method killnode, to manually kill the rosnode. Defaults to False.
            verbose (bool, optional): If set to true, logs info about the publisher behaviour. Defaults to True.
        """
        print("\n0\n")
        self.node_name = node_name
        print("\n1\n")
        self.anonymous = anonymous
        print("\n2\n")
        self.verbose = verbose
        print("\n3\n")
        self.topic = topic
        print("\n4\n")
        self.queue = queue
        print("\n5\n")
        self.data = getattr(message, data)
        print("\n5\n")
        self.signal = signal
        print("\n6\n")
        try:
            self.publisher = rospy.Publisher(self.topic, self.data, queue_size=self.queue)
            print("\nArrivato a prima\n")
            rospy.init_node(self.node_name, anonymous=self.anonymous, disable_signals=self.signal)
            print("\nArrivato a dopo\n")
            
            if self.verbose:
                #log when node starts (really useful for debugging)
                rospy.loginfo(f"Started publisher {self.node_name} to topic: {self.topic}")
            
        except rospy.ROSException:
            print("failed to initialize publisher!")
            pass
        
        
    def publish_once(self,message):
        """Publish a message once to the given topic

        Args:
            message (_type_): message of the same type of the data specified when instatiating the ROS pub object
        """
        
        try:
            self.publisher.publish(message) #publish the message
            if self.verbose:  
                rospy.loginfo(f"Node {self.node_name} published: {message}")
            return
                
        except rospy.ROSException:
            print("failed to publish!")
            return
    
    def publish_recur(self, message, rate_pub:int):
        """Publish a message to the given topic recurrently with a rate_pub frequency (Hz)

        Args:
            message (_type_): message of the same type of the data specified when instatiating the ROS pub object
            rate_pub (int): rate publishing frequency in Hertz
        """
        rate = rospy.Rate(rate_pub)
        try:
            while not rospy.is_shutdown():
                self.publisher.publish(message) #publish the message
                rate.sleep() #sleep for the amount setted with Rate
                
                if self.verbose:  
                    rospy.loginfo(f"Node {self.node_name} published: {message}")
                    
        except rospy.ROSInterruptException:
            print("publisher interruped")
            return
    
    def killnode(self):
        assert self.signal is not False, "Flag signal was set to False instead of True.\nUnable to kill node!"
        rospy.signal_shutdown("Node publisher was killed manually")
            
        