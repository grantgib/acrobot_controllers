%% Acrobot - Assume no input or external forces
clear; clc; close all;

%% Initialize Symbolics
syms px pz q1 q2 real
syms pxdot pzdot q1dot q2dot real
syms M Mp real
syms g real
syms L real
% point masses so no moment of inertia 
q = [px; pz; q1; q2];
qdot = [pxdot; pzdot; q1dot; q2dot];
n_q = length(q);

%% Mass positions
p_stance = [px; pz];
p1 = p_stance + [-(L/2)*sin(q1); (L/2)*cos(q1)];
p2 = p_stance + [-L*sin(q1); L*cos(q1)];
p3 = p2 + [(L/2)*sin(q1+q2); -(L/2)*cos(q1+q2)];
p_swing = p2 + [L*sin(q1+q2); -L*cos(q1+q2)];

J_stance = jacobian(p_stance,q);
Jdot_stance = jacobian(J_stance*qdot,q);

J_swing = jacobian(p_swing,q);
Jdot_swing = jacobian(J_swing*qdot,q);


%% Mass velocities
p1_dot = jacobian(p1,q)*qdot;
p2_dot = jacobian(p2,q)*qdot;
p3_dot = jacobian(p3,q)*qdot;

%% Kinetic Energy
K = (1/2)*(M)*(p1_dot'*p1_dot) + (1/2)*Mp*(p2_dot'*p2_dot) + (1/2)*M*(p3_dot'*p3_dot);
K = simplify(K);

%% Potential Energy
V1 = M*g*p1(2);
V2 = Mp*g*p2(2);
V3 = M*g*p3(2);
V = simplify(V1 + V2 + V3);

%% Mass Inertia Matrix
D = jacobian(jacobian(K,qdot),qdot);
D = simplify(D);        % simplify is important here!

%% C(q,q_dot)
n = size(D,2); % size of q
C = sym(zeros(n,n));    % initialize coriolis matrix
for i = 1:n
    for j = 1:n
        for k = 1:n
            C(i,j) = C(i,j) + (1/2) * qdot(k) * ( jacobian(D(i,j),q(k)) + ...
                jacobian(D(i,k),q(j)) - jacobian(D(j,k),q(i)) );
        end
    end
end
C = simplify(C);

%% Gravity Vectory
G = jacobian(V,q);
G = simplify(G);


%% Impact
tes = (-2.*(M + 2.*Mp)*cos(- q2)*q1dot + M*(q1dot+q2dot))/(-3.*M - 4.*Mp + 2.*M*cos(2.*( - q2)));
tes = simplify(subs(tes,[M,Mp],[5,10]))

wes = ((M - 4.*(M + Mp)*cos(2.*(- q2)))*q1dot + 2.*M*cos( - q2)*(q1dot+q2dot))/(-3.*M - 4.*Mp + 2.*M*cos(2.*( - q2)));
wes = simplify(subs(wes,[M,Mp],[5,10]))

De = [D, -J_swing'; J_swing, zeros(2,2)];
qplusF = De \ [D*qdot; zeros(2,1)];

qplus = simplify(subs(qplusF(1:4),[M,Mp,L,pxdot,pzdot],[5,10,1,0,0]));
qplus = qplus(3:4)

%% Generate Functions
% Kinematics
p_stance = subs(p_stance,[M,Mp,L,g],[5,10,1,9.8]);
p_stance_func = matlabFunction(p_stance,'Vars',{q});

J_stance = subs(J_stance,[M,Mp,L,g],[5,10,1,9.8]);
J_stance_func = matlabFunction(J_stance,'Vars',{q});

Jdot_stance = subs(Jdot_stance,[M,Mp,L,g],[5,10,1,9.8]);
Jdot_stance_func = matlabFunction(Jdot_stance,'Vars',{q,qdot});

p1 = subs(p1,[M,Mp,L,g],[5,10,1,9.8]);
p_M1_func = matlabFunction(p1,'Vars',{q});

p2 = subs(p2,[M,Mp,L,g],[5,10,1,9.8]);
p_Mp_func = matlabFunction(p2,'Vars',{q});

p3 = subs(p3,[M,Mp,L,g],[5,10,1,9.8]);
p_M2_func = matlabFunction(p3,'Vars',{q});

p_swing = subs(p_swing,[M,Mp,L,g],[5,10,1,9.8]);
p_swing_func = matlabFunction(p_swing,'Vars',{q});

J_swing = subs(J_swing,[M,Mp,L,g],[5,10,1,9.8]);
J_swing_func = matlabFunction(J_swing,'Vars',{q});

% Dynamics
D = subs(D,...
    [M,Mp,L,g],...
    [5,10,1,9.8]);
D_func = matlabFunction(D,'Vars',{q});

C = subs(C,...
    [M,Mp,L,g],...
    [5,10,1,9.8]);
C_func = matlabFunction(C,'Vars',{[q,qdot]});

G = subs(G,...
    [M,Mp,L,g],...
    [5,10,1,9.8]);
G_func = matlabFunction(G,'Vars',{q});

%% Save
M = 5;
Mp = 10;
L = 1;
g = 9.8;
save('loadDynamics',...
    'p_stance_func','p_M1_func','p_Mp_func','p_M2_func','p_swing_func',...
    'J_stance_func','Jdot_stance_func','J_swing_func',...
    'D_func','C_func','G_func',...
    'n_q','M','Mp','L','g');
disp("Dynamics Functions Saved!");
