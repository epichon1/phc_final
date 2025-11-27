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
from geometry_msgs.msg  import PoseStamped, TwistStamped
from geometry_msgs.msg  import TransformStamped
from sensor_msgs.msg    import JointState
from std_msgs.msg       import Header
from std_msgs.msg import Float64
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
        self.R0 = Reye()

        # Define the other points.
        self.pleft  = np.array([0.3, 0.5, 0.15])
        self.pright = np.array([-0.3, 0.5, 0.15])
        self.Rleft  = np.array([[0, 0, -1.0],
                  [1.0, 0, 0],
                  [0, -1.0, 0]])
        self.Rright = Reye()

        self.qlast = self.q0.copy()
        self.elast = np.zeros(6)

        # Pick the convergence bandwidth.
        self.lam = 20
        self.lam2 = 0.5
        self.gamma = 0.1
        # FIXME: WHAT DO YOU NEED TO DO TO INITIALIZE THE TRAJECTORY?

        # FIXME: REUSE THE PREVIOUS INVERSE KINEMATICS INITIALIZATION.


        ##############################################################
        # Setup the logistics of the node:
        # Add publishers to send the joint and task commands.  Also
        # add a TF broadcaster, so the desired pose appears in RVIZ.
        self.pubjoint = self.create_publisher(JointState, '/joint_states', 10)
        self.pubpose  = self.create_publisher(PoseStamped, '/pose', 10)
        self.pubtwist = self.create_publisher(TwistStamped, '/twist', 10)
        self.pubcond = self.create_publisher(Float64, '/condition', 10)
        self.tfbroad  = tf2_ros.TransformBroadcaster(self)

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
        if self.t > 8.0:
            self.future.set_result("Trajectory has ended")
            return
        #FIXME: IMPLEMENT THE TRAJECTORY.
        if self.t < 3.0:
            # Approach movement:
            (s0, s0dot) = goto(self.t, 3.0, 0.0, 1.0)

            pd = self.p0 + (self.pright - self.p0) * s0
            vd =           (self.pright - self.p0) * s0dot

            Rd = Reye()
            wd = np.zeros(3)

        else:
            # Pre-compute the path variables.  To show different
            # options, we compute the position path variable using
            # sinusoids and the orientation variable via splines.
            t = (self.t - 3.0) % 5
            if t < 2.5:
                (sl, sldot) = goto(t, 2.5, 0.0, 1.0)
                z = -0.15*np.cos((4*pi*t)/5)+0.3
                zdot = 0.15*(0.8)*np.sin((4*pi*t)/5)
                y = 0.5
                x = self.pright[0] + (self.pleft[0]-self.pright[0]) * sl
                xdot = (self.pleft[0]-self.pright[0]) * sldot
                pd = np.array([x,y,z])
                vd = np.array([xdot,0,zdot])

                (alpha,alphadot) = goto(t, 2.5,0,pi/2)
                (beta,betadot) = goto(t, 2.5,0,-pi/2)
                Rd = Rotz(alpha) @ Rotx(beta)
                wd = nz() * alphadot + Rotz(alpha) @ (nx() * betadot)
            else:
                (sr, srdot) = goto(t-2.5, 2.5, 0.0, 1.0)
                z = -0.15*np.cos((4*pi*t)/5)+0.3
                zdot = 0.15*(0.8)*np.sin((4*pi*t)/5)
                y = 0.5
                x = self.pleft[0] + (self.pright[0]-self.pleft[0]) * sr
                xdot = (self.pleft[0]-self.pright[0]) * srdot
                pd = np.array([x,y,z])
                vd = np.array([xdot,0,zdot])

                (alpha,alphadot) = goto(t-2.5, 2.5,pi/2,0)
                (beta,betadot) = goto(t-2.5, 2.5,-pi/2,0)
                Rd = Rotz(alpha) @ Rotx(beta)
                wd = nz() * alphadot + Rotz(alpha) @ (nx() * betadot)

        #FIXME: REUSE THE PREVIOUS INVERSE KINEMATICS UPDATE.
        qc = self.qlast.copy()
        elast = self.elast.copy()
        xdot = np.concatenate((vd, wd))

        (pc,Rc,Jv,Jw) = self.chain.fkin(qc)
        l = 0.4
        J = np.vstack((Jv, Jw))
        Jbar = np.diag([1/l,1/l,1/l,1,1,1])@J
        condition = np.linalg.cond(Jbar)

        u, s, vT = np.linalg.svd(J) 
        Ji = np.linalg.pinv(J)
        tau = repulsion(self.qlast, self.elbowchain, self.wristchain)
        qcdot = Ji @ (xdot + self.lam*elast) + (np.eye(7)-Ji@J) @ (5*tau)
        qc = qc + self.dt * qcdot
        
        errR = eR(Rd,Rc)
        errp = ep(pd, pc)
        self.elast = np.concatenate((errp, errR))
        self.qlast = qc.copy()


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
