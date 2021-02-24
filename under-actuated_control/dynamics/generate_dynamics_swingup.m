%% Acrobot - Assume no input or external forces
clear; clc; close all;

%% Initialize Symbolics
syms q1 q2 real
syms q1_dot q2_dot real
syms m1 m2 real
syms g real
syms L1 L2 Lcm1 Lcm2 real
syms I1 I2 real % moment of inertia of rod about COM
q = [q1; q2];
q_dot = [q1_dot; q2_dot];

%% Positions of Center of Mass w.r.t q
p_pivot = [0; 0];       % origin of pinned acrobot link
pcm1 = p_pivot + [Lcm1*cos(q1); Lcm1*sin(q1)];
pcm2 = p_pivot + [L1*cos(q1) + Lcm2*cos(q1+q2); L1*sin(q1) + Lcm2*sin(q1+q2)];

%% Velocities...
pcm1_dot = jacobian(pcm1,q)*q_dot;
pcm2_dot = jacobian(pcm2,q)*q_dot;   % chain rule


%% Kinetic Energy (Translational and  Rotational for each joint)
K1 = (1/2)*m1*(pcm1_dot(1)^2 + pcm1_dot(2)^2) + (1/2)*I1*(q1_dot)^2;   % note: the absolute angle velocity in the world frame is used for rotational kinetic energy. This is why it is q1+q2
K2 = (1/2)*m2*(pcm2_dot(1)^2 + pcm2_dot(2)^2) + (1/2)*I2*(q1_dot + q2_dot)^2;
K_total = K1 + K2;
K_total = simplify(K_total);

%% Potential Energy
V1 = m1*g*pcm1(2);
V2 = m2*g*pcm2(2);
V = V1 + V2;

%% Mass Inertia Matrix
D = jacobian(jacobian(K_total,q_dot),q_dot);
D = simplify(D);        % simplify is important here!
disp("Check Elements of D");
disp("D(1,1)"); disp(D(1,1))
disp("D(1,2)"); disp(D(1,2));
disp("D(2,1)"); disp(D(2,1));
disp("D(2,2)"); disp(D(2,2));
disp("---------------------------------------------------------------");

%% C(q,q_dot)
n = size(D,2); % size of q
C = sym(zeros(n,n));    % initialize coriolis matrix
for i = 1:n
    for j = 1:n
        for k = 1:n
            C(i,j) = C(i,j) + (1/2) * q_dot(k) * ( jacobian(D(i,j),q(k)) + ...
                jacobian(D(i,k),q(j)) - jacobian(D(j,k),q(i)) );
        end
    end
end
C = simplify(C);
disp("Check Elements of C");
disp("C(1,1)"); disp(C(1,1))
disp("C(1,2)"); disp(C(1,2));
disp("C(2,1)"); disp(C(2,1));
disp("C(2,2)"); disp(C(2,2));
disp("---------------------------------------------------------------");

%% Gravity Vectory
G = jacobian(V,q);
G = simplify(G);
disp("Check Elements of G");
disp("G(1)"); disp(G(1))
disp("G(2)"); disp(G(2));
disp("---------------------------------------------------------------");

%% Spong variables
% d11 = D(1,1);
% d22 = D(2,2);
% d12 = D(1,2);
% d21 = d12; % symmetric
% 
% H = simplify(C*q_dot);
% h1 = H(1);
% h2 = H(2);
% 
% phi1 = G(1);
% phi2 = G(2);
% 
% B = [0; 1]; % actuator is only at the knee

%% Generate Functions
D = subs(D,...
    [m1,m2,L1,L2,Lcm1,Lcm2,I1,I2,g],...
    [1,1,1,1,0.5,0.5,0.2,1,9.8]);
D_func = matlabFunction(D,'Vars',q);

C = subs(C,...
    [m1,m2,L1,L2,Lcm1,Lcm2,I1,I2,g],...
    [1,1,1,1,0.5,0.5,0.2,1,9.8]);
C_func = matlabFunction(C,'Vars',[q,q_dot]);

G = subs(G,...
    [m1,m2,L1,L2,Lcm1,Lcm2,I1,I2,g],...
    [1,1,1,1,0.5,0.5,0.2,1,9.8]);
G_func = matlabFunction(G,'Vars',q);

%% Linearization for LQR
disp('============================');
syms u real;
B = [0; 1];
xdot = simplify([q_dot; D \ (-C*q_dot - G' + B*u)])

A = jacobian(xdot,[q;q_dot]);
B = jacobian(xdot,u);

A = subs(A,[q1,q2,q1_dot,q2_dot],[pi/2,0,0,0]);
B = subs(B,[q1,q2,q1_dot,q2_dot],[pi/2,0,0,0]);
Q = [1000, -500, 0, 0;
    -500, 1000, 0, 0;
    0, 0, 1000, -500;
    0, 0, -500, 1000];
R = 0.5;
K_lqr = lqr(double(A),double(B),Q,R)

% lqr about the state [q1-pi/2; q2; q_dot]

%% Save
m1 = 1;
m2 = 1;
L1 = 1;
L2 = 1;
Lcm1 = 0.5;
Lcm2 = 0.5;
I1 = 0.2;
I2 = 1;
g = 9.8;
save('loadDynamics','D_func','C_func','G_func',...
    'm1','m2','L1','L2','Lcm1','Lcm2','I1','I2','g',...
    'K_lqr');



