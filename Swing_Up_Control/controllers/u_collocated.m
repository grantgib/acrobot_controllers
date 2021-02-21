function tau = u_collocated(x,p)
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
    
    if q1dot > 0
        q2_des = p.alpha;
    else
        q2_des = -p.alpha;
    end
    
    v2 = -p.kd*(q2dot) - p.kp*(q2 - q2_des);
    
    d11 = D(1,1);
    d12 = D(1,2);
    d21 = D(2,1);
    d22 = D(2,2);
    h1 = H(1);
    h2 = H(2);
    phi1 = G(1);
    phi2 = G(2);
    
    d2bar = d22 - (d21*d12)/d11;
    h2bar = h2 - (d21*h1)/d11;
    phi2bar = phi2 - (d21*phi1)/d11;
    
    tau(i) = d2bar*v2 + h2bar + phi2bar;
end
end