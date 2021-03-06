% Script for simple kinematic control

clear;
close all;
clc;

include_namespace_dq; 
addpath('./singularity_trajectory')
% Initialize V-REP interface
vi = DQ_VrepInterface;
vi.disconnect_all(); 
vi.connect('127.0.0.1',19997);
vi.start_simulation();

% Initialize VREP Robots
ur_robot = FEpVrepRobot('UR10',vi);

% Load DQ Robotics kinematics
ur  = ur_robot.kinematics();

% maximum joint ranges (deg): (q1..q7)
q_min = [-pi/2 -pi/2 -pi/2 -2*pi -pi/2 -2*pi];

q_max = [pi/2 0 pi/2 2*pi pi/2 2*pi];

goal = [1.4907 -1.0102 1.0535 -2.8032 1.5705 0.0722];
load('Traj_tele_3.mat')
%% Contol Loop

xd = ur.fkm(goal);
e = zeros(8,1);
e(1) = 1.0;
disp("Starting control loop:");
it = 1;
while(norm(e)>0.05) 
   xd = DQ([out(it, :)']);
   q = vi.get_joint_positions(ur_robot.joint_names); 
   disp("Current joint positions ");
   q
   x = ur.fkm(q);
   e = vec8(x - xd);
   J = ur.pose_jacobian(q);
   
   J_geom = geomJ(ur,q);
   [u, s, v] = svd(J);
   s(6, 6)
   % -------- Singularity avoidance ----------
   
   
   
   
   % -----------------------------------------
   u = -0.01 * pinv(J) * e;
   q = q + u;
   disp("------------------");
   ur_robot.send_q_to_vrep(q);
   pause(0.1)
   it = it + 1;
end

x = ur.fkm(goal);


% x.translation.q
% x.P
T = DQuaternionToMatrix(vec8(x)');
transfW_rf = [[-1 0 0; 0 1 0; 0 0 -1], [0,0,1]'; 0 0 0 1];
transfW_rf\T
%% End V-REP
disp("Control finished");
vi.stop_simulation();
vi.disconnect();





