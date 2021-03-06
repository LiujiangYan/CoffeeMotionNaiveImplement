# CoffeeMotionNaiveImplement

Prerequisite:  
1. rvctools (robotics toolbox in matlab, seeing http://petercorke.com/Robotics_Toolbox.html)  
2. TOPP (Time Optimal Path Parameterization, seeing https://github.com/quangounet/TOPP)
  
There exists two approachs to prevent spilling coffee out of the cup during the robot arm motion,  
1. adjust the orientation of the end effector based on the kinematics  
2. limit the kinematics 
  
Here we present an naive implementation of the first approach. We decouple the 6-axis UR5 robot to two parts: the first three axis determines the position in cartesian space and last three axis determines the orientation. Also, we apply the TOPP package to get the parameterized path (trajectory).  
  
The whole process consists of three parts:  
1. (ctraj of UR5.m) giving the start and goal point in the cartesian space, computes the straight path* based on the first 3-axis robot. Differentiate the joint variable along the path and output the data (q0, q1, qd0, qd1) for TOPP.  
2. (TOPP coffee motion.py) giving the formatted path data from above, and the velocity and acceleration constraints (for each axis in rad/s), computes the time optimal parameterized path (time array, joint variable/velocity/acceleration) in csv files.  
3. (coffee motion.m) computes the complete six axis path.

* the reason we use the straight path is that in such a case the only kinematics variable we care is the cartesian acceleration. Otherwise we have to consider the Coriolis Force(an inertial force that acts on objects that are in motion relative to a rotating reference frame).  
* we have not yet tested the program in simulation or real robot and still ongoing refine it.  
* a serious problem now is that the inverse kinematics function ikine6s is not for ur5 configuration.
