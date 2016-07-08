%the start and goal state in Cartesian space
start = [0.1, 0.6, 0.5];
goal = [-0.4, 0.2, 0.2];
%set the step of the path in Cartesian space
t = [0:0.01:2]';

%construct the first three axis of UR5
L(1) = Link('d', 0.182, 'a', 0, 'alpha', pi/2);
L(2) = Link('d', 0, 'a', -0.620, 'alpha', 0);
L(3) = Link('d', 0, 'a', -0.559, 'alpha', 0);
ur5_fore = SerialLink(L, 'name', 'ur5');

%get the transition matrix of the initial states
Tstart = transl(start);
Tgoal = transl(goal);

%plan the path in Cartesian Space
%get the inverse kinematics
Ts = ctraj(Tstart, Tgoal, length(t));
qc = ur5_fore.ikine3(Ts);

%uncomment this line to see how the path goes
%ur5_fore.plot(qc);

%the distance of the path
ds = norm(goal - start)/length(t);
%get the differential of the joint variable via distance
qc_1 = gradient(qc(:,1))./ds;
qc_2 = gradient(qc(:,2))./ds;
qc_3 = gradient(qc(:,3))./ds;
qc_d = [qc_1'; qc_2'; qc_3']';

%process the data for following TOPP interpolate function
%input (q0, q1, qd0, qd1, ds)
qc_for = qc(1:end-1,:);
qc_lat = qc(2:end,:);
qc_d_for = qc_d(1:end-1,:);
qc_d_lat = qc_d(2:end,:);

%save in csv
input = [qc_for, qc_lat, qc_d_for, qc_d_lat];
csvwrite('input.csv',input)