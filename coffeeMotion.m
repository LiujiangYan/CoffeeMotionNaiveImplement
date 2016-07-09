%construct the first three axis of UR5
L(1) = Link('d', 0.182, 'a', 0, 'alpha', pi/2);
L(2) = Link('d', 0, 'a', -0.620, 'alpha', 0);
L(3) = Link('d', 0, 'a', -0.559, 'alpha', 0);
ur5_fore = SerialLink(L, 'name', 'ur5_fore');

%read the parameterized path (trajectory) from TOPP
%the time array
%joint variable, joint velocity, joint acceleration
time = csvread('data/time.csv');
jointVariable = csvread('data/jointVariable.csv');
jointVel = csvread('data/jointVel.csv');
jointAccl = csvread('data/jointAccl.csv');
%ur5_fore.plot(jointVariable);

%construct the full UR5
ur5_L(1) = Link('d', 0.182, 'a', 0, 'alpha', pi/2);
ur5_L(2) = Link('d', 0, 'a', -0.620, 'alpha', 0);
ur5_L(3) = Link('d', 0, 'a', -0.559, 'alpha', 0);
ur5_L(4) = Link('d', 0, 'a', 0, 'alpha', pi/2);
ur5_L(5) = Link('d', 0, 'a', 0, 'alpha', -pi/2);
ur5_L(6) = Link('d', 0, 'a', 0, 'alpha', 0);

ur5_full = SerialLink(ur5_L, 'name', 'ur5-6axis');
ur5_full.ikineType = 'puma';

%get the transition matrix of the first three joint variable
T = ur5_fore.fkine(jointVariable);
%iterate the time array and compute the orientation through the Cartesian
%acceleration
for i=1:length(time)
    %the jacobian matrix
    J = ur5_fore.jacob0(jointVariable(i,:));
    %the product of the differential of jacobian matrix and the joint
    %velocity
    Jd = ur5_fore.jacob_dot(jointVariable(i,:), jointVel(i,:));
    %get the cartesian acceleration of end effector
    cAccel = J*jointAccl(i,:)' + Jd;
    
    %get the orientation
    pose_x = atan2(cAccel(2),(cAccel(3)+9.81));
    pose_y = atan2(cAccel(1),(cAccel(3)+9.81));
    %get the complete transition matrix and update
    %T(:,:,i) = T(:,:,i) * trotx(pose_x) * troty(pose_y);
    T(1:3,1:3,i) = rotx(pose_x) * roty(pose_y);
end

%from the transition matrix compute the 6 axis joint variable trajectory
qc = ur5_full.ikine6s(T);
%plot the trajectory
ur5_full.plot(qc)
