function dxdt = acro_pointmass_ode(t,x,p)
% Extract Inputs
D_func = p.D_func;
C_func = p.C_func;
G_func = p.G_func;
J_func = p.J_stance_func;
Jdot_func = p.Jdot_stance_func;
n_q = p.n_q;
q = x(1:n_q);
qdot = x(n_q+1:end);

% Calculate Matrices
D = D_func(q);
C = C_func([q, qdot]);
G = G_func(q)';
J = J_func(q);
Jdot = Jdot_func([q,qdot]);

De = [D, -J'; J, zeros(2,2)];
Ce = [C; Jdot];
Ge = [G; zeros(2,1)];

qddot_GRF = De \ (-Ce*qdot - Ge);
qddot = qddot_GRF(1:n_q);

dxdt = [qdot; qddot];

end