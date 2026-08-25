%% DH transformation matrices and direct kinematics of a serial robot

clear all
clc

%% Define symbolic variables

syms alpha d a theta

%% Number of joints of ROBOT

N=7;

%% Stop at i frame if needed
% Use this if need to stop the direct kinematics on a previous frame

STOP = 7;

%% Insert DH table of parameters of YUMI ROBOT
% alpha a d theta
DHTABLE = [
          (-pi/2)     -0.03     0.166    sym('q1');
          (pi/2)    0.03       0       sym('q2');
          (-pi/2)    0.0405    0.2515   sym('q3');
          (-pi/2)     0.0405        0    sym('q4');
          (-pi/2)     0.027       0.265  sym('q5');
          (pi/2)    -0.027        0    sym('q6');
            0        0         0.036    sym('q7')
           ];

         
%% Build the general Denavit-Hartenberg trasformation matrix

TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build transformation matrices for each link
% First, we create an empty cell array

A = cell(1,N);

% For every row in 'DHTABLE' we substitute the right value inside
% the general DH matrix

for i = 1:STOP

    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
    
    T = eye(4);
    
    T = T*A{i};
    T = simplify(T);
    fprintf('\n');
    fprintf('From %d to %d\n', i-1, i);  
    fprintf('A%d \n', i); 
    RM = T
end

%% Direct kinematics
fprintf('----------------------------------------------------\n');
fprintf('----------------------------------------------------\n\n');
disp('Direct kinematics of X robot in symbolic form (simplifications may need some time)')
disp(['Number of joints N=',num2str(N)])
disp(['Stopping evaluation at ',num2str(STOP)])

% Note: 'simplify' may need some time

%eye is the identity matrix%
T = eye(4);

for i=1:STOP 
    T = T*A{i}; 
    T = simplify(T);
end

% output TN matrix
disp('TN matrix')
T0N = T
    
% output xN axis
disp('xN axis, 1 column')
n=T(1:3,1)

% output yN axis
disp('yN axis, 2 column')
s=T(1:3,2)

% output zN axis
disp('zN axis, 3 column')
a=T(1:3,3)

% output ON position
disp('position, 4 column')
p = T(1:3,4)

%% Substitute the DH variable with givens values
fprintf('----------------------------------------------------\n');
fprintf('----------------------------------------------------\n\n');
fprintf('Substituting qi with the following set of');
VALUES = {0,-pi/2,0,-pi/2,0,0,0};
values = VALUES
SubsMatrix = subs(T, {sym('q1'),sym('q2'),sym('q3'),sym('q4'),sym('q5'),sym('q6'),sym('q7')}, VALUES);
SubsMatrix = SubsMatrix

%% end
