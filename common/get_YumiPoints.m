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

function [Yumi]=get_YumiPoints(q)

    global robot
   
    Yumi=zeros(3,7);
    ph=[0;0;0;1];

 base = robot.Tbase*ph;
 Yumi(:,1)=base(1:3)
 
    A0=robot.Tbase*robot.transMat(q(1),0,-1,-0.03,robot.d1);
    A1=A0*robot.transMat(q(2),0,1,0.03,0);
    A2=A1*robot.transMat(q(3),0,-1,0.0405,robot.d3);
    A3=A2*robot.transMat(q(4),0,-1,0.0405,0);
    A4=A3*robot.transMat(q(5),0,-1,0.027,robot.d5);
    A5=A4*robot.transMat(q(6),0,1,-0.027,0);
    A6=A5*robot.transMat(0,1,0,0,robot.d7);
    
    mats ={A0, A1,A2,A3,A4,A5, A6};
    for i=1:7
        pi_ = mats{i}*ph;
        Yumi(:,i+1)=pi_(1:3);
    end
end
