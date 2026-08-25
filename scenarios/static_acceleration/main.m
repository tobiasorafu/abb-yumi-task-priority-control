addpath(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'common'));
%      Copyright 2020 Maram Khatib
%  
%      Licensed under the Apache License, Version 2.0 (the "License");
%      you may not use this file except in compliance with the License.
%      You may obtain a copy of the License at
%      http://www.apache.org/licenses/LICENSE-2.0
%  
%      Unless required by applicable law or agreed to in writing, software
%      distributed under the License is distributed on an "AS IS" BASIS,
%      WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
%      See the License for the specific language governing permissions and
%      limitations under the License.
% Modified for the ABB YuMi (left arm) group project by Tobias Orafu, Emediong Moffat, and Kofoworola Oyeniyi.

%%% The robot has a positional and an inequality pointing tasks with αd = 5
%%% For control points of collision avoidance, the one dimensional task definition in used
%%% Only one intrest control point is considered for body collision avoidance 
%%% Two danger regions are considered

clc; 
clear;
%%% initialization

d1=0.166;     % distance from base frame to second joint frame 
d3=0.2515;     % distance from second joint frame to fourth joint frame
d5=0.265;    % distance from fourth joint frame to sixth joint frame 
d7=0.036;   % distance from sixth joint frame to EE frame; EE in the tip of KUKA without auxiliary addition

Zd =[1,0,0];% parallel to x-axis, for desired pointing task 

%initial position

% ---- Solve for a starting configuration that already sits on the path ----
% Rather than starting at q=[0;...;0] (which is generally nowhere near the
% ellipse's t=0 point), solve for q1 such that the end-effector starts
% at Task_yumi(0), using damped-least-squares IK (Jacobian + damped_pinv,
% both already in this codebase). This removes the large initial
% convergence transient/loop that would otherwise show up in the EE path.

q_seed=[0.3;-0.3;0.3;0.5;0.3;0.5;0]; % nonzero seed; avoids the singular all-zero pose
%q1=[0;0;0;0;0;0;0];
dq1=[0;0;0;0;0;0;0];%initial velocity
ddq1=[0;0;0;0;0;0;0];%initial acceleration
    
global robot
robot=YUMI_LWR(d1,d3,d5,d7,q_seed,dq1,Zd);% temporary, just to drive the IK loop

[p_start,~,~] = Task_yumi(0); % desired EE position at t=0 on the ellipse

q_ik=q_seed;
ikMaxIter=300;
ikTol=1e-5;
for ikIter=1:ikMaxIter
    robot.set_q_and_qdot(q_ik,dq1);
    p_cur=robot.computeEnd_effectorPosition;
    err=p_start-p_cur;
    if norm(err)<ikTol
        break;
    end
    J=robot.computeJacobianPos;
    dq=damped_pinv(J,0.05,1e-8,1e-10)*err;
    q_ik=q_ik+dq;
end
q1=q_ik;

% Re-construct the robot cleanly at the solved starting configuration so
% PrevJacobianPos/PrevJacobianOri/PrevJacobianControlPoints are consistent
% with q1 (not left over from the IK iterations), avoiding a spurious
% one-step jump in the finite-difference dJacobian terms.
robot=YUMI_LWR(d1,d3,d5,d7,q1,dq1,Zd);


obsControlPoints = zeros(1,8);% 8 control points
numberOfObstacles=3;
p_obs = zeros(numberOfObstacles,3);
p_obs(1,:)=[ -0.002 0.08 0];   % cross the path
 p_obs(2,:)=[-0.08; 0.24;  0.10];   % up one 
 p_obs(3,:)=[ 0.04; 0.3; -0.15];   % down one
                              
T=0.001; %sampling time
Ttot=6*pi; % total simulation time
n=7; %number of joints
I=eye(n);

p0_ee=robot.computeEnd_effectorPosition;%initial Cartesian position

[~,Ori_z]=robot.computeEnd_effectorOrientation;
orientation1_ee=Zd*Ori_z;%cos alpha

%current configuration
q=q1;
dq_prref=dq1;
ddq_prref=ddq1;

%controller gains
Kp0=40;%positional task
Kd0=20;%positional task

Krp1=5;%pointing task
Krd1=2;%pointing task

[p,dp,ddp] = Task_yumi(0);%initial Cartesian point

desiredAcc0=zeros(3,1);
desiredVelocity0=zeros(3,1);
desiredPosition0=p;
desiredAcc1=0;
desiredVelocity1=0;

alpha_d=8;% desired alpha for pointing task 5
desiredorientation1=cos(deg2rad(alpha_d));

%save quantities
Size=round(Ttot/T);
QV=zeros(Size,7);
dQV=zeros(Size,7);
ddQV=zeros(Size,7);
distanceControlPointsV=zeros(Size,8);
    

T0d=zeros(Size,3);
e0=zeros(Size,3);
Ze1=zeros(Size,3);
Pe1=zeros(Size,3);

T1d=zeros(Size,1);
e1=zeros(Size,1);

extTime=zeros(Size,1);

obscontrolPointMin=zeros(Size,1);
dpControlpointmin=zeros(Size,3);
controlPointMin=ones(Size,1)*-1;
velocityControlPointMin=zeros(Size,1);
distanceControlPointsMin=ones(Size,1);
distanceControlPointsMin=distanceControlPointsMin*.11;

obscontrolPointMin2=zeros(Size,1);
dpControlpointmin2=zeros(Size,3);
controlPointMin2=zeros(Size,1);
velocityControlPointMin2=zeros(Size,1);
distanceControlPointsMin2=ones(Size,1);
distanceControlPointsMin2=distanceControlPointsMin2*.11;

distanceEEMin=ones(Size,1);
distanceEEMin=distanceEEMin*.11;

%%% control loop
i=1;
for t=0:T:Ttot
   
    %TASK 0: ellipse with EE
    robot.set_q_and_qdot(q,dq_prref);
    p0_ee=robot.computeEnd_effectorPosition;
    dp0_ee=robot.computeEnd_effectorPosVelocity;
    A0=robot.computeJacobianPos;
    dJacobianPos= (A0 - robot.PrevJacobianPos)/T;
    
 [p,dp,ddp] = Task_yumi(t);
 
    desiredPosition0=p;
	desiredVelocity0=dp;
    desiredAcc0=ddp;
    e0(i,:)=desiredPosition0-p0_ee;
    ev0=desiredVelocity0-dp0_ee;
   
	b0= Kp0*(e0(i,:)')+Kd0*ev0;
   	b0= b0 + desiredAcc0 - dJacobianPos*dq_prref;

    %TASK 1: EE pointing 
    [~,Ori_z]=robot.computeEnd_effectorOrientation;
    Ze1(i,:)=Ori_z;
    orientation1_ee=Zd*Ori_z;
    alpha=rad2deg(acos(orientation1_ee));
    [~,A1]=robot.computeJacobianOri(Zd,Ori_z);
    dJacobianOri= (A1 - robot.PrevJacobianOri)/T;
    
    desiredAcc1=0;
    desiredVelocity1=0;
    e1(i,1)=desiredorientation1-orientation1_ee;%pointing error
    ev1=desiredVelocity1-robot.computeEnd_effectorOriVelocity(Zd);
	b1= Krp1*e1(i,1)+Krd1*ev1;
   	b1= b1 + desiredAcc1- (dJacobianOri*dq_prref);

	desiredPosition0=p;
    desiredorientation1=cos(deg2rad(alpha_d));
    
    %save current quantities
    QV(i,:)=q;
    dQV(i,:)=dq_prref;
    ddQV(i,:)=ddq_prref;
    
    T0d(i,:)=p0_ee;
    T1d(i,:)=orientation1_ee;
    
   %TASK 2: EE obstacle avoidance
    [dp2,dmin,velocity2,nn] = repulsiveEE(p0_ee,p_obs);
    b2=dp2;
    A2=A0;
    distanceEEMin(i,1)=dmin;
 
   %TASK 3: links obstacle avoidance
    ControlPoints=robot.compute_control_points_position;
    %Pe1(i,:)=ControlPoints(:,5)';
    Pe1(i,:)=p0_ee';
    Jac_ControlPoints=robot.compute_control_points_jacobian;
    PrevJac_ControlPoints=robot.PrevJacobianControlPoints;
    [distanceControlPoints,obsControlPoints,dpControlPoints,velocityControlPoints,nnControlPoints] = repulsiveControlPoints(ControlPoints,p_obs);
    distanceControlPointsV(i,:)= distanceControlPoints;
    
    maxVelocity=0;
    nearestControlPoint=0;% critical control point to be considered in the collsion avoidance
    for c=1:1:8 
       if velocityControlPoints(1,c)>maxVelocity
           nearestControlPoint=c;
           maxVelocity=velocityControlPoints(1,c);
       end
    end
    
%%% Arranging tasks priority
    A=A0;
    idx=3;%task dimention
    b=b0+b2;% positional task + EE cllision avoidance
    
    % collision avoidance of nearest Control Point    
if nearestControlPoint~=0
        dotJc1 = (Jac_ControlPoints(:,:,nearestControlPoint) - PrevJac_ControlPoints(:,:,nearestControlPoint))/T;
        Ac1= nnControlPoints(:,nearestControlPoint)'*Jac_ControlPoints(:,:,nearestControlPoint);
        nnControlPoints(:,nearestControlPoint);
        bc1= nnControlPoints(:,nearestControlPoint)'*dpControlPoints(:,nearestControlPoint);
        dotJc1=nnControlPoints(:,nearestControlPoint)'*dotJc1;
        bc1= bc1 - dotJc1*dq_prref;

        %saving for drawing
        dpControlpointmin(i,:)= dpControlPoints(:,nearestControlPoint)';
        controlPointMin(i,1)=nearestControlPoint;
        obscontrolPointMin(i,1)=obsControlPoints(1,nearestControlPoint);
        velocityControlPointMin(i,1)=velocityControlPoints(1,nearestControlPoint);
        distanceControlPointsMin(i,1)=distanceControlPoints(1,nearestControlPoint);

        flag=0;

 if distanceControlPointsV(i,nearestControlPoint)<= 0.05 % close danger threshold

    if   distanceControlPointsV(i,nearestControlPoint) <= distanceEEMin(i,1)

            A=[Ac1;A];
            idx=[1,idx];
            b=[bc1;b];
    else
            A=[A;Ac1];
            idx=[idx,1];
            b=[b;bc1];
    end

    flag=1;
 end

end

 if  e1(i,1)> 0 %1e-4 %%pointing error

     A=[A;A1];
     idx=[idx,1];
     b=[b;b1]; 

 end

if nearestControlPoint~=0 

 if  flag==0 

     A=[A;Ac1];% including link collision avoidance as least priority 
     idx=[idx,1];
     b=[b;bc1];      
 end
end

%%% damping parameters for Jacobian inverse 
eps=1e-1;
lambda=0.5e-1;

[Q,R]=qr(A',0);
X=tasksPriorityMatrix(R,idx,lambda,eps);% task priority matrix

Kdamp=5;% joint damping gain
Jtotal = A;
JtotalInv= Q*pinv(R');

ddq_prref=JtotalInv*X*b +  ((I-(JtotalInv*Jtotal))* -Kdamp*dq_prref);% commanded acceleration  

q= q + dq_prref*T+ddq_prref*0.5*T*T; %%current position
dq_prref=dq_prref+ddq_prref*T; %%current velocity

robot.set_update(Zd);

i=i+1;

    if (mod(t,1)==0)
        t;
    end
    
end

disp('finish');
