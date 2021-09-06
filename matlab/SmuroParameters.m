%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%                      Smuro Robotics Parameters                      %%%
%%%                          ZEFFIRETTI, HIESH                          %%%
%%%                   Beijing Institute of Technology                   %%%
%%%                zeffiretti@bit.edu.cn, hiesh@mail.com                %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% what you may concern, namely, key varibles are as following
%       SList: Screw axes Si of the joints in a space frame, in the format
%              of a matrix with the screw axes as the columns.
%       M:     The home configuration of the end-effector, or the head link
%       BList: The joint screw axes in the end-effector frame when the
%              manipulator is at the home position, in the format of a 
%              matrix with the screw axes as the columns.
%       Mlist: List of link com frames {i} relative to {i-1} at the home 
%              position.
%       GList: List of spatial inertia matrices Gi of the links.
%       g:     Gravity vector g.

%% Parameters
% pitch and yaw axis w.r.t. space frame
w_pitch = [0; -1; 0];
w_yaw   = [0;  0; 1];

% wi refers to roration axis of joint i, w.r.t space frame
w1 = w_pitch;
w2 = w_pitch;
w3 = w_yaw;
w4 = w_yaw;
w5 = w_pitch;
w6 = w_yaw;
w7 = w_pitch;

% qi refers to a point on joint i, w.r.t space frame
q1 = [0,       0,  0.0265]';
q2 = [0.0368,  0,  0.0433]';
q3 = [0.0368,  0,  0.0433]';
q4 = [0.0768,  0,  0.0433]';
q5 = [0.0768,  0,  0.0433]';
q6 = [0.13042, 0,  0.0433]';
q7 = [0.14255, 0,  0.0433]';

% com List of links center of mass position, w.r.t link frame
com = [[0.009,     0.008,    0]',...
       [0.0,      -0.002,    0]',...
       [0.02,      0.0,      0]',...
       [-0.00085, -0.002,    0]',...
       [0.0317,   -0.001,    0]',...
       [0.0125,    0.0,     -0.0032]',...
       [0.018,     0.0,      0]',...
       [0.0,       0.0,      0]'];
% Mi refers to com configuration w.r.t link frame i, corresponding with
%    com[i]
M1 = [eye(3), com(:,1); zeros(1,3), 1];
M2 = [eye(3), com(:,2); zeros(1,3), 1];
M3 = [eye(3), com(:,3); zeros(1,3), 1];
M4 = [eye(3), com(:,4); zeros(1,3), 1];
M5 = [eye(3), com(:,5); zeros(1,3), 1];
M6 = [eye(3), com(:,6); zeros(1,3), 1];
M7 = [eye(3), com(:,7); zeros(1,3), 1];
M8 = [eye(3), com(:,8); zeros(1,3), 1];
% q_{i-1,i} link farme i origin position w.r.t link frame i-1, defined with
%           joint xyz properity in urdf file
q01 = [0         0       0.0265]';
q12 = [0.0368    0.0168  0]';
q23 = [0         0       0]';
q34 = [0.04      0       0]';
q45 = [0         0       0]';
q56 = [0.05362   0       0]';
q67 = [0.01213   0       0]';
q78 = [0         0       0]';
% rpy_{i-1,i} rpy angle between link frames, defined with joint rpy
%             properity in urdf file
rpy01 = [pi/2      0     0 ];
rpy12 = [0         0     0.4286];
rpy23 = [-pi/2     0    -0.4286];
rpy34 = [0         0     0];
rpy45 = [pi/2      0     0];
rpy56 = [-pi/2     0     0];
rpy67 = [pi/2      0     0];
rpy78 = [0         0     0];
% T_{i-1,i} refers to transformation matrix between link frames
% euler(alpha, beta, gamma) is equivalent to rpy(gamma, beta, alpha), which
% explains the use of function fliplr
T01 = [eul2rotm(fliplr(rpy01)), q01;...
       0,0,0,1];
T12 = [eul2rotm(fliplr(rpy12)), q12;...
       0,0,0,1];
T23 = [eul2rotm(fliplr(rpy23)), q23;...
       0,0,0,1];
T34 = [eul2rotm(fliplr(rpy34)), q34;...
       0,0,0,1];
T45 = [eul2rotm(fliplr(rpy45)), q45;...
       0,0,0,1];
T56 = [eul2rotm(fliplr(rpy56)), q56;...
       0,0,0,1];
T67 = [eul2rotm(fliplr(rpy67)), q67;...
       0,0,0,1];
T78 = [eul2rotm(fliplr(rpy78)), q78;...
       0,0,0,1];
% verify transformation matrix calculaiton
T08 = T01*T12*T23*T34*T45*T56*T67*T78;
% M_{i-1,i} refers varibles in modern robotics library, link com frames {i}
%           relative to {i-1} at the home position
M01 = T01 * M1;
M12 = M1 \ T12 * M2;
M23 = M2 \ T23 * M3;
M34 = M3 \ T34 * M4;
M45 = M4 \ T45 * M5;
M56 = M5 \ T56 * M6;
M67 = M6 \ T67 * M7;
M78 = M7 \ T78 * M8;

% mass and G matrix
mass_scale = 1;
inertia_scale = 1;
mass = mass_scale * [0.0086, 0.002, 0.14, 0.002, 0.0417, 0.0092, 0.0078];
G1 = diag([inertia_scale*[  3e-6,     3e-6,       2e-6], mass(1)*ones(1,3)]);
G2 = diag([inertia_scale*[1.5e-7,     4e-7,     1.8e-7], mass(2)*ones(1,3)]);
G3 = diag([inertia_scale*[  5e-5,     5e-5,       4e-5], mass(3)*ones(1,3)]);
G4 = diag([inertia_scale*[1.8e-7,   1.8e-7,       1e-8], mass(4)*ones(1,3)]);
G5 = diag([inertia_scale*[  7e-6,     9e-6,       8e-6], mass(5)*ones(1,3)]);
G6 = diag([inertia_scale*[4.5e-7,   3.9e-7,     3.2e-7], mass(6)*ones(1,3)]);
G7 = diag([inertia_scale*[9.3e-7,  1.07e-6,    1.19e-6], mass(7)*ones(1,3)]);

% w.r.t body
w1b = w_yaw;
w2b = w_yaw;
w3b = w_pitch;
w4b = w_pitch;
w5b = w_yaw;
w6b = w_pitch;
w7b = w_yaw;

q1b = [-0.142549,    -0.0168114,       0]';
q2b = [-0.10575,      0,               0]';
q3b = q2b;
q4b = [-0.06575,      0,               0]';
q5b = q4b;
q6b = [-0.01213,      0,               0]';
q7b = [0,             0,               0]';


% key var 01
SList = [ScrewToAxis(q1, w1, 0),...
         ScrewToAxis(q2, w2, 0),...
         ScrewToAxis(q3, w3, 0),...
         ScrewToAxis(q4, w4, 0),...
         ScrewToAxis(q5, w5, 0),...
         ScrewToAxis(q6, w6, 0),...
         ScrewToAxis(q7, w7, 0)];

% key var 02
M = [1,  0,  0,  0.14253;...
     0,  0, -1,  0      ;...
     0,  1,  0,  0.0433 ;...
     0,  0,  0,  1      ];

% key var 03
BList = [ScrewToAxis(q1b, w1b, 0),...
         ScrewToAxis(q2b, w2b, 0),...
         ScrewToAxis(q3b, w3b, 0),...
         ScrewToAxis(q4b, w4b, 0),...
         ScrewToAxis(q5b, w5b, 0),...
         ScrewToAxis(q6b, w6b, 0),...
         ScrewToAxis(q7b, w7b, 0)];
     
%key var 04
MList = cat(3, M01, M12, M23, M34, M45, M56, M67, M78);

% key var 05
GList = cat(3, G1, G2, G3, G4, G5, G6, G7);

% key var 06
g = [0; 0; -9.8];

% unused actually
% M_B = [[1;0;0;0],[0;0;1;0],[0;-1;0;0],[-0.142547;-0.0433114;0;1]];

%% Parameters Brief
disp('################Smuro Parameters Brief################')
disp(' Screw axes Si of the joints in a space frame SList: ') 
disp(SList)
disp('-----------------------------------------------------')
disp(' The home configuration of the end-effector M: ')
disp(M)
disp('-----------------------------------------------------')
disp(' The joint screw axes in the end-effector frame when')
disp('     the manipulator is at the home position BList: ')
disp(BList)
disp('-----------------------------------------------------')
disp(' List of link com frames {i} relative to {i-1} at the')
disp('      home position MList: ')
disp(MList)
disp('-----------------------------------------------------')
disp(' List of spatial inertia matrices GList: ')
disp(GList)
disp('-----------------------------------------------------')
disp(' Gravity vector g: ')
disp(g)
disp('################Smuro Parameters End.#################')
%% Test
% test parameters
thetalist = [0.6,-0.3,0,0,-0.3,0,0]';
dthetalist = zeros(7,1);
ddthetalist = zeros(7,1);
Ftip = [0; 0; 0; 0; 0; 0];
%% Forward Kinematics Test
fks = FKinSpace(M,SList,thetalist);
%% Inverse Kinematics Test
random_theta = thetalist + 0.1*rand(7,1);
[ik_theta, success] = IKinSpace(SList, M, fks, random_theta, 1e-5, 1e-5);
%% Inverse Dynamics Test
% Torque needed to maintain initial configuration
taulist = InverseDynamics(thetalist, dthetalist, ddthetalist, g, ...
                        Ftip, MList, GList, SList);
%% Forward Dynamics Test
fddthetalist = ForwardDynamics(thetalist, dthetalist, taulist, ...
                                       g, Ftip, MList, GList, SList);
