%% Calculates the Jacobian for differential kinematics of a Robot

clear all
clc

%% Define symbolic variables

syms alpha d a theta

%% Number of joints of ROBOT

N=7;

%% Insert DH table of parameters of ROBOT
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
     

%% Setting the type of the joints (0 primastic, 1 revolute)
JointsType = [1, 1, 1, 1, 1, 1, 1];

%% Build the general Denavit-Hartenberg trasformation matrix

TDH = [ cos(theta) -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);
        sin(theta)  cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);
          0             sin(alpha)             cos(alpha)            d;
          0               0                      0                   1];

%% Build the general Jacobian

J = sym(zeros(6,N));
      
%% Build transformation matrices for each link
% First, we create an empty cell array

A = cell(1,N);

% For every row in 'DHTABLE' we substitute the right value inside
% the general DH matrix

T = eye(4);
%Contains the combined rotation matrices at each step
% i.e   COMBINED{2} = A{1}A{2}
%       COMBINED{i} = A{1}A{2}....A{i}
COMBINED = cell(1,N);

alpha = DHTABLE(1,1);
a = DHTABLE(1,2);
d = DHTABLE(1,3);
theta = DHTABLE(1,4);
A{1} = subs(TDH);
COMBINED{1} = A{1};

for i = 2:N

    alpha = DHTABLE(i,1);
    a = DHTABLE(i,2);
    d = DHTABLE(i,3);
    theta = DHTABLE(i,4);
    A{i} = subs(TDH);
    
    T = eye(4);
    
    T = T*A{i};
    T = simplify(T);
    
    COMBINED{i} = COMBINED{i-1}*T;
end

%eye is the identity matrix%
T = eye(4);

%% Calculating the transformation matrix

TSTEP = cell(1,N);

for i=1:N
    T = T*A{i}; 
    T = simplify(T);
    TSTEP{i} = T;
end

for i=1:N
    if i == 1
        TAUX = sym(eye(4));
        CAUX = sym(eye(4));
    else
        TAUX = TSTEP{i-1};
        CAUX = COMBINED{i-1};
    end
    
    % if prismatic joint
    if JointsType(i) == 0
        JP= simplify([CAUX(1,3),CAUX(2,3),CAUX(3,3)]);
        JO = [0, 0, 0]';
    % if revolute joint
    else
       
        
        Zi = [CAUX(1,3),CAUX(2,3),CAUX(3,3)];
        
        PE = [  T(1,4); 
                T(2,4);
                T(3,4)];
        
        Pi = [  TAUX(1,4); 
                TAUX(2,4); 
                TAUX(3,4)];
        
        JP = simplify(cross(Zi, PE-Pi));
        JO = simplify([TAUX(1,3), TAUX(2,3), TAUX(3,3)]);
        
        J(1,i) = JP(1);
        J(2,i) = JP(2);
        J(3,i) = JP(3);
        J(4,i) = JO(1);
        J(5,i) = JO(2);
        J(6,i) = JO(3);
        
    end
end

JACOBIAN = J

V = [   J(1,:);
        J(2,:);
        J(3,:)]


W = [   J(4,:);
        J(5,:);
        J(6,:)]
%% end
