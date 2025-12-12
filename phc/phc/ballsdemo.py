"""balldemo.py

   Simulate a non-physical ball and publish as a visualization marker
   array to RVIZ.

   Node:      /balldemo
   Publish:   /visualization_marker_array   visualization_msgs.msg.MarkerArray

"""

import rclpy
import numpy as np

from asyncio                    import Future
from rclpy.node                 import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from rclpy.time                 import Duration

from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray

from utils.TransformHelpers     import *


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = 0.1
        self.ballsnum = 2

        # list of vectors (one for each ball) instead of singular (for just one ball)
        self.p = []
        self.v = []
        self.a = []
        
        # original
        # self.p = np.array([0.0, 0.0, self.radius])
        # self.v = np.array([1.0, 0.1,  5.0       ])
        # self.a = np.array([0.0, 0.0, -9.81      ])

        # loop through all balls and create a random position/velocity
        # but an acceleration consistent with gravity
        for i in range(self.ballsnum):

            self.p.append(np.array(
                [np.random.uniform(-0.5, 0.5),
                np.random.uniform(-0.5, 0.5),
                np.random.uniform(0.25, 1.0)]))
            
            self.v.append(np.array(
                [np.random.uniform(-0.5, 0.5),
                np.random.uniform(-0.5, 0.5),
                np.random.uniform(1.0, 5.0)]))
            
            self.a.append(np.array([0.0, 0.0, -9.81]))
        '''
        # Create the sphere marker once (updating only the stamp/position).
        diam        = 2 * self.radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.p)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        # a = 0.8 is slightly transparent!
        # Create the marker array message.
        self.markerarray = MarkerArray(markers = [self.marker])
        '''
        # create the sphere marker once for each ball with a unique position and color
        diam        = 2 * self.radius
        self.markers = []

        for i in range(self.ballsnum):
            marker = Marker()
            marker.header.frame_id  = "world"
            marker.header.stamp     = self.get_clock().now().to_msg()
            marker.action           = Marker.ADD
            marker.ns               = "point"
            marker.id               = i
            marker.type             = Marker.SPHERE
            marker.pose.orientation = Quaternion()
            marker.pose.position    = Point_from_p(self.p[i])
            marker.scale            = Vector3(x = diam, y = diam, z = diam)
            marker.color            = ColorRGBA(
                                        r = np.random.uniform(0.25, 1.0),
                                        g = np.random.uniform(0.25, 1.0),
                                        b = np.random.uniform(0.25, 1.0),
                                        a = 0.8
                                      )
            self.markers.append(marker)

        # change from original: self.markers is already a list
    
        self.markerarray = MarkerArray(markers = self.markers)
        ##############################################################
        # Setup the logistics of the node:
        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        self.pub = self.create_publisher(
            MarkerArray, '/visualization_marker_array', quality)

        # Set up the timer to update at 100Hz, with (t=0) occuring in
        # the first update cycle (dt) from now.
        self.dt    = 0.01                       # 100Hz.
        self.t     = -self.dt                   # Seconds since start
        self.now   = self.get_clock().now()     # ROS time since 1970
        self.timer = self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, 1/self.dt))

    # Shutdown
    def shutdown(self):
        # Destroy the timer, then shut down the node.
        self.timer.destroy()
        self.destroy_node()


    # Update - send a new joint command every time step.
    def update(self):
        # Increment time.  We do so explicitly to avoid system jitter.
        self.t   = self.t   + self.dt
        self.now = self.now + rclpy.time.Duration(seconds=self.dt)

        ##############################################################
        # COMPUTE THE TRAJECTORY AT THIS TIME INSTANCE.

        # Integrate the velocity, then the position.
        # self.v += self.dt * self.a
        # self.p += self.dt * self.v

        # integrate velocity and position for all balls 
        for i in range(self.ballsnum):
            self.v[i] += self.dt * self.a[i]
            self.p[i] += self.dt * self.v[i]

            if self.p[i][2] < self.radius:
                self.p[i][2] = self.radius + (self.radius - self.p[i][2])
                self.v[i][2] *= -1.0
                self.v[i][0] *= -1.0   # Change x just for the fun of it!

            # Update the message and publish.
            self.markers[i].header.stamp  = self.now.to_msg()
            # self.marker.pose.position = Point_from_p(self.p)
            self.markers[i].pose.position = Point_from_p(self.p[i])
            
        '''
        # Check for a bounce - not the change in x velocity is non-physical.
        if self.p[2] < self.radius:
            self.p[2] = self.radius + (self.radius - self.p[2])
            self.v[2] *= -1.0
            self.v[0] *= -1.0   # Change x just for the fun of it!
        '''
        # Update the ID number to create a new ball and leave the
        # previous balls where they are.
        #####################
        # self.marker.id += 1
        #####################


        ##############################################################
        # Finish by publishing the data
        
        self.pub.publish(self.markerarray)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the node ends.
    future = Future()

    # Initialize the demo node.
    node = DemoNode('pirouette', future)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the node is
    # complete (as signaled by the future object).
    rclpy.spin_until_future_complete(node, future)

    # Report the reason for shutting down.
    if future.done():
        node.get_logger().info("Stopping: " + future.result())
    else:
        node.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    node.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
