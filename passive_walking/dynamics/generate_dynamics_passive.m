%% Generate Dynamics
clear; clc; close all;
% Dynamics generated using model from:
%   * http://underactuated.mit.edu/simple_legs.html
%   * Schematic is in directory
sym_param = 0;
if sym_param
    syms g L m m_h real
else
    m = 5; 
    m_h = 10;
    g = 9.81;
    L = 1;
end

syms px pz px_dot pz_dot real
syms qst qsw qst_dot qsw_dot real

% Body coordinates
q = [qst; qsw];
q_dot = [qst_dot; qsw_dot];

% Floating base coordinates
qfb = [px; pz; qst; qsw];
qfb_dot = [px_dot; pz_dot; qst_dot; qsw_dot];

xfb = [qfb; qfb_dot];

%% Floating Base Model
% Positions
pst = [px; pz];
p1 = pst + [-(L/2)*sin(qst); (L/2)*cos(qst)];
p2 = pst + [-L*sin(qst); L*cos(qst)];
p3 = p2 + [(L/2)*sin(qsw); -(L/2)*cos(qsw)];
psw = p2 + [(L)*sin(qsw); -(L)*cos(qsw)];

% Velocities
p1_dot = jacobian(p1,qfb)*qfb_dot;
p2_dot = jacobian(p2,qfb)*qfb_dot;
p3_dot = jacobian(p3,qfb)*qfb_dot;

% Kinetic Energy
K1 = (1/2)*m*(p1_dot'*p1_dot);
K2 = (1/2)*m_h*(p2_dot'*p2_dot);
K3 = (1/2)*m*(p3_dot'*p3_dot);
K = K1 + K2 + K3;

% Potential Energy
P1 = m*g*p1(2);
P2 = m_h*g*p2(2);
P3 = m*g*p3(2);
P = P1 + P2 + P3;

% Mass Inertia Matrix
D = jacobian(jacobian(K,qfb_dot),qfb_dot);
D = simplify(D);        % simplify is important here!

% C(q,q_dot)
n = size(D,2); % size of q
C = sym(zeros(n,n));    % initialize coriolis matrix
for i = 1:n
    for j = 1:n
        for k = 1:n
            C(i,j) = C(i,j) + (1/2) * qfb_dot(k) * ( jacobian(D(i,j),qfb(k)) + ...
                jacobian(D(i,k),qfb(j)) - jacobian(D(j,k),qfb(i)) );
        end
    end
end
C = simplify(C);

% Gravity Vectory
G = jacobian(P,qfb)';
G = simplify(G);

% xdot
% [D,   -Jst']   [qddot]   [C*qdot     ]   [G]   [0]
% [Jst,     0] * [ Fst ] + [Jstdot*qdot] + [0] = [0]
Jst = simplify( jacobian(pst,qfb) );
Jstdot = simplify(jacobian(Jst*qfb_dot,qfb_dot));
qfb_ddot_Fst = [D,-Jst';Jst, zeros(2,2)] \ ( -[C*qfb_dot;Jstdot*qfb_dot] - [G;zeros(2,1)]);
qfb_ddot = qfb_ddot_Fst(1:4);

x_dot = [qfb_dot; qfb_ddot];

%% Impact Map Derivation
invD = simplify(inv(D));
Jsw = jacobian(psw,qfb);
qfb_dot_plus = (eye(4) - invD*Jsw'*inv(Jsw*invD*Jsw')*Jsw) * qfb_dot;
qfb_dot_plus = simplify(qfb_dot_plus);

x_impact = [qfb; 
    0; 0; qfb_dot_plus(3:4)];

%% Reset Map
R = [0, 1; 1, 0];
qfb_switch = [psw(1); psw(2); R*q];
qfb_dot_switch = [0; 0; R*q_dot];
x_switch = [qfb_switch; qfb_dot_switch];

%% Generate Functions
% positions
pst_func = matlabFunction(pst,'Vars',{qfb});
p1_func = matlabFunction(p1,'Vars',{qfb});
p2_func = matlabFunction(p2,'Vars',{qfb});
p3_func = matlabFunction(p3,'Vars',{qfb});
psw_func = matlabFunction(psw,'Vars',{qfb});

% ode function
x_dot_func = matlabFunction(x_dot,'Vars',{xfb});
x_impact_func = matlabFunction(x_impact,'Vars',{xfb});
x_switch_func = matlabFunction(x_switch,'Vars',{xfb});

save('load_dynamics','L',...
    'pst_func','p1_func','p2_func','p3_func','psw_func',...
    'x_dot_func','x_impact_func','x_switch_func');
disp('Dynamics Saved!');

