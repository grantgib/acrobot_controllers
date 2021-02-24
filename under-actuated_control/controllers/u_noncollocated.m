function tau = u_noncollocated(x,p)
for i = 1:size(x,2)
    xi = x(:,i);
    q1 = xi(1); q2 = xi(2);
    q = [q1; q2];
    q1dot = xi(3); q2dot = xi(4);
    qdot = [q1dot; q2dot];
    
    D = p.D_func(q(1),q(2));
    C = p.C_func(q(1),q(2),qdot(1),qdot(2));
    H = C*qdot;
    G = p.G_func(q(1),q(2))';
    
    q1_des = pi/2;
    q1dot_des = 0;
    q1ddot_des = 0;
    
    d11 = D(1,1);
    d12 = D(1,2);
    d21 = D(2,1);
    d22 = D(2,2);
    h1 = H(1);
    h2 = H(2);
    phi1 = G(1);
    phi2 = G(2);
    
    v1 = q1ddot_des - p.kd*(q1dot - q1dot_des) - p.kp*(q1 - q1_des);
    
    d1bar = d21 - (d22*d11)/d12;
    h1bar = h2 - (d22*h1)/d12;
    phi1bar = phi2 - (d22*phi1)/d12;
    
    tau(i) = d1bar*v1 + h1bar + phi1bar;
end
end