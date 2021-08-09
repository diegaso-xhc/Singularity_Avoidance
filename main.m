% Script for simple kinematic control

clear;
close all;
clc;

include_namespace_dq; 

% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); 
vi.connect('127.0.0.1',19997);
vi.start_simulation();

% Initialize VREP Robots
fep_vreprobot = FEpVrepRobot('UR10',vi);

% Load DQ Robotics kinematics
fep  = fep_vreprobot.kinematics();

% maximum joint ranges (deg): (q1..q7)
%       -166.0031 -101.0010 -166.0031 -176.0012 -166.0031  -1.0027  -166.0031
q_min = [-2*pi -2*pi -2*pi -2*pi -2*pi -2*pi];

%        166.0031  101.0010  166.0031 -3.9992   166.0031   215.0024  166.0031
q_max = [-2*pi -2*pi -2*pi -2*pi -2*pi -2*pi]*(-1);

   
%goal = [0, 0, 0, 0, 0, 0];
goal = [0, 0, 0, 0, 0, 0];
%fep_vreprobot.send_q_to_vrep(goal);

%goal = goal + [pi/2 pi/2 0 pi/2 0 0];

%%
xd = fep.fkm(goal);
e = zeros(8,1);
e(1) = 1.0;
disp("Starting control loop:");

while(norm(e)>0.05)  
   q = vi.get_joint_positions(fep_vreprobot.joint_names); 
   disp("Current joint positions ");
   q
   x = fep.fkm(q);
   e = vec8(x -xd)
   J = fep.pose_jacobian(q);
   u = -0.01 * pinv(J) * e;
   q = q + u;
   disp("------------------");
   fep_vreprobot.send_q_to_vrep(q);
end

x = fep.fkm(goal);
x.translation.q
x.P

%% End V-REP
disp("Control finished");
vi.stop_simulation();
vi.disconnect();





