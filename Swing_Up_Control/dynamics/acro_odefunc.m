function dxdt = acro_odefunc(t,x,p)
% Extract Inputs
D_func = p.D_func;
C_func = p.C_func;
G_func = p.G_func;
q = x(1:2);
q_dot = x(3:end);

% Calculate Matrices
D = D_func(q(1),q(2));
C = C_func(q(1),q(2),q_dot(1),q_dot(2));
G = G_func(q(1),q(2))';
B = [0; 1];             % The motors act directly at the joints independently

% t
u = compute_ctrl(t,x,p);
% u = 0;
dxdt = [q_dot;
    D \ (-C*q_dot - G + B*u(1))];
end