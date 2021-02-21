%% Acrobot - Assume no input or external forces
clear; clc; close all;

%% Initialize Symbolics
syms q1 q2 real
syms q1_dot q2_dot real
syms m1 m2 real
syms g real
syms L1 L2 Lcm1 Lcm2 real
syms Jcm1 Jcm2
q = [q1; q2];
q_dot = [q1_dot; q2_dot];

%% Positions of Center of Mass w.r.t q
p_pivot = [0; 0];       % origin of pinned acrobot link
pcm2 = p_pivot + [Lcm2*cos(q2); Lcm2*sin(q2)];
pcm1 = p_pivot + [L2*cos(q2) + Lcm1*cos(q1+q2); L2*sin(q2) + Lcm1*sin(q1+q2)];

%% Velocities...
pcm2_dot = jacobian(pcm2,q)*q_dot;   % chain rule
pcm1_dot = jacobian(pcm1,q)*q_dot;

%% Kinetic Energy (Translational and  Rotational for each joint)
K2 = (1/2)*m2*(pcm2_dot(1)^2 + pcm2_dot(2)^2) + (1/2)*Jcm2*(q2_dot)^2;
K1 = (1/2)*m1*(pcm1_dot(1)^2 + pcm1_dot(2)^2) + (1/2)*Jcm1*(q1_dot+q2_dot)^2;   % note: the absolute angle velocity in the world frame is used for rotational kinetic energy. This is why it is q1+q2
K_total = K1+K2;
K_total = simplify(K_total);

%% Potential Energy
m_total = m1+m2;
pcm_total = simplify((m1/m_total)*pcm1 + (m2/m_total)*pcm2);
% V = m1*g*pcm1(2) + m2*g*pcm2(2);
V = simplify(m_total*g*pcm_total(2));

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
disp("This is correct, book is wrong, the q1 should be replaced with q2");
disp("---------------------------------------------------------------");


%% Generate Functions
D = subs(D,...
    [Jcm1,Jcm2,L1,L2,Lcm1,Lcm2,m1,m2],...
    [0.03,0.03,1,1,0.2,0.2,0.3,0.3]);
D_func = matlabFunction(D,'Vars',q);

C = subs(C,...
    [Jcm1,Jcm2,L1,L2,Lcm1,Lcm2,m1,m2],...
    [0.03,0.03,1,1,0.2,0.2,0.3,0.3]);
C_func = matlabFunction(C,'Vars',[q,q_dot]);

G = subs(G,...
    [Jcm1,Jcm2,L1,L2,Lcm1,Lcm2,m1,m2,g],...
    [0.03,0.03,1,1,0.2,0.2,0.3,0.3,9.81]);
G_func = matlabFunction(G,'Vars',q);
save('loadDynamics_Appendix','D_func','C_func','G_func');



