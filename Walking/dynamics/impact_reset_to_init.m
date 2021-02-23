function x_plus = impact_reset_to_init(x_minus,p)
    q_minus = x_minus(1:p.n_q,1);
    qdot_minus = x_minus(p.n_q+1:end,1);
    
%     J = p.J_swing_func(q_minus);
%     D = p.D_func(q_minus);
%     
%     q_plus = q_minus;
%     qdot_plus_F = inv([D, -J'; J, zeros(2,2)]) * [D*qdot_minus; zeros(2,1)];
%     
%     qdot_plus = qdot_plus_F(1:p.n_q,1);
%     x_plus = [q_plus; qdot_plus];
%     
%     deltaF = -((J/D)*J') \ J * [eye(2,2); zeros(2,2)]
%     
%     deltadqbar = D \ J' * deltaF + [eye(2,2); zeros(2,2)]
%     
%     R = [-1,0; 2 0];
%     
%     R * q_minus(3:4)
%     
%     [R, zeros(2,2)]*deltadqbar*qdot_minus(3:4)
    

    M = p.M;
    Mp = p.Mp;
    q1 = q_minus(3);
    q2 = q_minus(4);
    q1dot = qdot_minus(3);
    q2dot = qdot_minus(4);
    
    q1dot_plus = (-2.*(M + 2.*Mp)*cos(- q2)*q1dot + M*(q1dot+q2dot))/(-3.*M - 4.*Mp + 2.*M*cos(2.*( - q2)));
    q2dot_plus = ((M - 4.*(M + Mp)*cos(2.*(- q2)))*q1dot + 2.*M*cos( - q2)*(q1dot+q2dot))/(-3.*M - 4.*Mp + 2.*M*cos(2.*( - q2)));
    
    q2dot_plus = q2dot_plus - q1dot_plus;
    
    q1plus = q1 + q2;
    q2plus = -q2;
    
    pst = p.p_swing_func(q_minus);
    x_plus = [pst; q1plus; q2plus; 0; 0; q1dot_plus; q2dot_plus];
    
    
    
    
    
end