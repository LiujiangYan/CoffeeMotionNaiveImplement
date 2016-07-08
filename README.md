# CoffeeMotionNaiveImplement

There exists two approachs to prevent spilling coffee out of the cup during the robot arm motion,  
1. adjust the orientation of the end effector based on the kinematics  
2. limit the kinematics  
  
Here we present an naive implementation of the first approach. We decouple the 6-axis UR5 robot to two parts: the first three axis determines the position in cartesian space and last three axis determines the orientation. Also, we apply the TOPP package to get the parameterized path (trajectory).  

