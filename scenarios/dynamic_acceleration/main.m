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

%%% Acceleration-level task-priority control with moving obstacles
clc;
clear;

d1=0.166;
d3=0.2515;
d5=0.265;
d7=0.036;
Zd=[1,0,0];

q_seed=[0.3;-0.3;0.3;0.5;0.3;0.5;0];
dq1=zeros(7,1);
ddq1=zeros(7,1);

global robot
robot=YUMI_LWR(d1,d3,d5,d7,q_seed,dq1,Zd);
[p_start,~,~]=Task_yumi(0);
q_ik=q_seed;
for ikIter=1:300
    robot.set_q_and_qdot(q_ik,dq1);
    err=p_start-robot.computeEnd_effectorPosition;
    if norm(err)<1e-5, break; end
    J=robot.computeJacobianPos;
    q_ik=q_ik+damped_pinv(J,0.05,1e-8,1e-10)*err;
end
q1=q_ik;
robot=YUMI_LWR(d1,d3,d5,d7,q1,dq1,Zd);

numberOfObstacles=3;
p_obs0=zeros(numberOfObstacles,3);
p_obs0(1,:)=[-0.002 0.08 0];
p_obs0(2,:)=[-0.08 0.24 0.10];
p_obs0(3,:)=[0.04 0.3 -0.15];
p_obs=moving_obstacles(0,p_obs0);

T=0.001;
Ttot=6*pi;
I=eye(7);
q=q1;
dq_prref=dq1;
ddq_prref=ddq1;
Kp0=40; Kd0=20;
Krp1=5; Krd1=2;
alpha_d=8;
desiredorientation1=cos(deg2rad(alpha_d));

Size=round(Ttot/T);
QV=zeros(Size,7); dQV=zeros(Size,7); ddQV=zeros(Size,7);
distanceControlPointsV=zeros(Size,8);
T0d=zeros(Size,3); e0=zeros(Size,3); Ze1=zeros(Size,3); Pe1=zeros(Size,3);
T1d=zeros(Size,1); e1=zeros(Size,1);
obscontrolPointMin=zeros(Size,1); dpControlpointmin=zeros(Size,3);
controlPointMin=ones(Size,1)*-1; velocityControlPointMin=zeros(Size,1);
distanceControlPointsMin=ones(Size,1)*.11;
distanceEEMin=ones(Size,1)*.11;
p_obsHistory=zeros(Size,numberOfObstacles,3);

i=1;
for t=0:T:Ttot
    p_obs=moving_obstacles(t,p_obs0);
    p_obsHistory(i,:,:)=reshape(p_obs,[1,numberOfObstacles,3]);

    robot.set_q_and_qdot(q,dq_prref);
    p0_ee=robot.computeEnd_effectorPosition;
    dp0_ee=robot.computeEnd_effectorPosVelocity;
    A0=robot.computeJacobianPos;
    dJacobianPos=(A0-robot.PrevJacobianPos)/T;

    [p,dp,ddp]=Task_yumi(t);
    e0(i,:)=p-p0_ee;
    ev0=dp-dp0_ee;
    b0=Kp0*e0(i,:)'+Kd0*ev0+ddp-dJacobianPos*dq_prref;

    [~,Ori_z]=robot.computeEnd_effectorOrientation;
    Ze1(i,:)=Ori_z;
    orientation1_ee=Zd*Ori_z;
    [~,A1]=robot.computeJacobianOri(Zd,Ori_z);
    dJacobianOri=(A1-robot.PrevJacobianOri)/T;
    e1(i,1)=desiredorientation1-orientation1_ee;
    ev1=-robot.computeEnd_effectorOriVelocity(Zd);
    b1=Krp1*e1(i,1)+Krd1*ev1-dJacobianOri*dq_prref;

    QV(i,:)=q; dQV(i,:)=dq_prref; ddQV(i,:)=ddq_prref;
    T0d(i,:)=p0_ee; T1d(i,:)=orientation1_ee;

    [dp2,dmin,~,~]=repulsiveEE(p0_ee,p_obs);
    b2=dp2;
    distanceEEMin(i,1)=dmin;

    ControlPoints=robot.compute_control_points_position;
    Pe1(i,:)=p0_ee';
    Jac_ControlPoints=robot.compute_control_points_jacobian;
    PrevJac_ControlPoints=robot.PrevJacobianControlPoints;
    [distanceControlPoints,obsControlPoints,dpControlPoints,velocityControlPoints,nnControlPoints]=repulsiveControlPoints(ControlPoints,p_obs);
    distanceControlPointsV(i,:)=distanceControlPoints;

    [maxVelocity,nearestControlPoint]=max(velocityControlPoints);
    if maxVelocity==0, nearestControlPoint=0; end

    A=A0; idx=3; b=b0+b2;
    flag=0;
    if nearestControlPoint~=0
        dotJc1=(Jac_ControlPoints(:,:,nearestControlPoint)-PrevJac_ControlPoints(:,:,nearestControlPoint))/T;
        Ac1=nnControlPoints(:,nearestControlPoint)'*Jac_ControlPoints(:,:,nearestControlPoint);
        bc1=nnControlPoints(:,nearestControlPoint)'*dpControlPoints(:,nearestControlPoint);
        bc1=bc1-nnControlPoints(:,nearestControlPoint)'*dotJc1*dq_prref;

        dpControlpointmin(i,:)=dpControlPoints(:,nearestControlPoint)';
        controlPointMin(i,1)=nearestControlPoint;
        obscontrolPointMin(i,1)=obsControlPoints(1,nearestControlPoint);
        velocityControlPointMin(i,1)=velocityControlPoints(1,nearestControlPoint);
        distanceControlPointsMin(i,1)=distanceControlPoints(1,nearestControlPoint);

        if distanceControlPointsV(i,nearestControlPoint)<=0.05
            if distanceControlPointsV(i,nearestControlPoint)<=distanceEEMin(i,1)
                A=[Ac1;A]; idx=[1,idx]; b=[bc1;b];
            else
                A=[A;Ac1]; idx=[idx,1]; b=[b;bc1];
            end
            flag=1;
        end
    end

    if e1(i,1)>0
        A=[A;A1]; idx=[idx,1]; b=[b;b1];
    end
    if nearestControlPoint~=0 && flag==0
        A=[A;Ac1]; idx=[idx,1]; b=[b;bc1];
    end

    [Q,R]=qr(A',0);
    X=tasksPriorityMatrix(R,idx,0.5e-1,1e-1);
    JtotalInv=Q*pinv(R');
    ddq_prref=JtotalInv*X*b+(I-JtotalInv*A)*(-5*dq_prref);
    q=q+dq_prref*T+0.5*ddq_prref*T*T;
    dq_prref=dq_prref+ddq_prref*T;

    robot.set_update(Zd);
    i=i+1;
end

disp('finish');
