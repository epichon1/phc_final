'''hw7p2.py

   This is a placeholder and skeleton code for HW7 Problem 2.

   Insert your HW7 P1 code and edit for this problem!

'''
import rclpy
import numpy as np
import tf2_ros

from math               import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from asyncio            import Future
from rclpy.node         import Node
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from geometry_msgs.msg  import PoseStamped, TwistStamped, Point, Vector3, Quaternion
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header
from std_msgs.msg import Float64
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray
from hw7code.repulsion import repulsion

# Grab the Utilities
from utils.TransformHelpers     import *
from utils.TrajectoryUtils      import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain


#
#   Trajectory Generator Node Class
#
#   This inherits all the standard ROS node stuff, but adds an
#   update() method to be called regularly by an internal timer and a
#   shutdown method to stop the timer.
#
#   Arguments are the node name and a future object (to force a shutdown).
#
class TrajectoryNode(Node):
    # Initialization.
    def __init__(self, name, future):
        # Initialize the node and store the future object (to end).
        super().__init__(name)
        self.future = future

        ##############################################################
        # INITIALIZE YOUR TRAJECTORY DATA!

        # Define the list of joint names MATCHING THE JOINT NAMES IN THE URDF!
        self.jointnames=['theta1','theta2','thetaX', 'theta3','theta4','theta5','theta6']

        # Set up the kinematic chain object.
        self.chain = KinematicChain(self, 'world', 'tip', self.jointnames)
        self.elbowchain=KinematicChain(self,'world','elbow',self.jointnames[0:4])
        self.wristchain=KinematicChain(self,'world','wrist',self.jointnames[0:5])

        # Define the matching initial joint/task positions.
        self.q0 = np.radians(np.array([0, 90,0, -90, 0, 0, 0]))
        self.qcenter = np.array([-pi/4,-pi/4, pi/2, -pi/2, 0, 0,0])
        self.p0= np.array([0.0, 0.55, 1.0])
        self.pos = self.p0.copy()
        self.R0 = Reye()
        self.ball_c_pos = [np.array([0.0, 0.0, 0.0]), np.array([0.0, 0.0, 0.0])]
        #self.pos_wrist = self.wristchain.fkin(self.q0)

        # Define the other points.
        self.pleft  = np.array([0.3, 0.5, 0.15])
        self.pright = np.array([-0.3, 0.5, 0.15])
        self.Rleft  = np.array([[0, 0, -1.0],
                  [1.0, 0, 0],
                  [0, -1.0, 0]])
        self.Rright = Reye()

        self.qlast = self.q0.copy()
        self.elast = np.zeros(6)
        self.ewlast = np.zeros(6)

        # Pick the convergence bandwidth.
        self.lam = 20
        self.lam2 = 0.5
        self.gamma = 0.1


        ##############################################################
        # Setup the logistics of the node:
        # Add publishers to send the joint and task commands.  Also
        # add a TF broadcaster, so the desired pose appears in RVIZ.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose  = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist = self.create_publisher(TwistStamped, '/twist', 10)
        self.pubcond = self.create_publisher(Float64, '/condition', 10)
        self.tfbroad  = tf2_ros.TransformBroadcaster(self)

        quality = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)

        self.subball = self.create_subscription(MarkerArray, '/visualization_marker_array',self.ball_tracking, quality)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

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
        #FIXME: IMPLEMENT THE TRAJECTORY.
        # Approach movement:
        qc = self.qlast.copy()
        elast = self.elast.copy()
        ewlast = self.ewlast.copy()

        t = (self.t) % 5

        Rd = Reye()
        wd = np.zeros(3)
        for i in range(len(self.pos)):
            if abs(self.pos[i]-self.ball_c_pos[0][i]) < 1e-3:
                self.pos[i] = self.ball_c_pos[0][i]
            
        pd,vd = goto(t, 5.0,self.pos,self.ball_c_pos[0])

        (pc_wrist, Rc_wrist, Jv_wrist, Jw_wrist) = self.wristchain.fkin(qc[0:5])
        J_wrist = np.vstack((Jv_wrist, Jw_wrist))
        J_wrist = np.hstack((J_wrist, np.zeros((6, 2))))

        
        ball_vec = self.ball_c_pos[1] - pc_wrist
        if np.linalg.norm(ball_vec) > 1e-6:
            d = ball_vec / np.linalg.norm(ball_vec)
        else:
            d = Rc_wrist[:,1]

        y_axis = Rc_wrist[:,1]
        errR_wrist = np.cross(y_axis, d)
        errp_wrist = np.zeros(3)
        vd_wrist = np.zeros(3)
        wd_wrist = self.lam2 * errR_wrist
        xdot_wrist = np.concatenate((vd_wrist, wd_wrist))
        
        xdot = np.concatenate((vd, wd))

        (pc,Rc,Jv,Jw) = self.chain.fkin(qc)
        l = 0.4
        J = np.vstack((Jv, Jw))
        Jbar = np.diag([1/l,1/l,1/l,1,1,1])@J
        condition = np.linalg.cond(Jbar)


        sigma = np.linalg.svd(J, compute_uv=False)
        minsv = min(sigma)

        lam_damp = 0.01 + 0.2 * np.exp(-5 * minsv)
        Ji_wrist = J_wrist.T @ np.linalg.inv(J_wrist @ J_wrist.T + (lam_damp)*np.eye(6))
        Ji = J.T @ np.linalg.inv(J @ J.T + (lam_damp)*np.eye(6))
        qcdot = Ji @ (xdot + self.lam*elast) + (np.eye(7) - Ji @ J) @ Ji_wrist @ (xdot_wrist + self.lam*ewlast)
        #(np.eye(7)-Ji@J) @ (self.lam2*(qc - self.qcenter))
        #qcdot = Ji_wrist @ (xdot_wrist + self.lam*ewlast) + (np.eye(7) - Ji_wrist @ J_wrist) @ Ji @ (xdot + self.lam*elast)

        errR = eR(Rd,Rc)
        errp = ep(pd, pc)

        # errp = np.clip(errp, -MAX_ERR, MAX_ERR)
        # errR = np.clip(errR, -MAX_ERR, MAX_ERR)

        qc = qc + self.dt * qcdot

        self.elast = np.concatenate((errp, errR))
        self.ewlast = np.concatenate((errp_wrist, errR_wrist))
        self.qlast = qc.copy()
        self.pos = pc.copy()


        ##############################################################
        # Finish by publishing the data (joint and task commands).
        #  qc and qcdot = Joint Commands  as  /joint_states  to view/plot
        #  pd and Rd    = Task pos/orient as  /pose & TF     to view/plot
        #  vd and wd    = Task velocities as  /twist         to      plot
        header=Header(stamp=self.now.to_msg(), frame_id='world')
        self.pubcond.publish(Float64(data=condition))
        self.pubjoint.publish(JointState(
            header=header,
            name=self.jointnames,
            position=qc.tolist(),
            velocity=qcdot.tolist()))
        self.pubpose.publish(PoseStamped(
            header=header,
            pose=Pose_from_Rp(Rd,pd)))
        self.pubtwist.publish(TwistStamped(
            header=header,
            twist=Twist_from_vw(vd,wd)))
        self.tfbroad.sendTransform(TransformStamped(
            header=header,
            child_frame_id='desired',
            transform=Transform_from_Rp(Rd,pd)))

    def ball_tracking(self, msg):
        self.ball_c_pos = []
        for marker in msg.markers:
            pos = marker.pose.position
            self.ball_c_pos.append(np.array([pos.x,pos.y,pos.z]))
        return
#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Create a future object to signal when the trajectory ends.
    future = Future()

    # Initialize the trajectory generator node.
    trajectory = TrajectoryNode('trajectory', future)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory is
    # complete (as signaled by the future object).
    rclpy.spin_until_future_complete(trajectory, future)

    # Report the reason for shutting down.
    if future.done():
        trajectory.get_logger().info("Stopping: " + future.result())
    else:
        trajectory.get_logger().info("Stopping: Interrupted")

    # Shutdown the node and ROS.
    trajectory.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
