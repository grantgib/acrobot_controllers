function tau = compute_ctrl(t,x,p)
if p.lqr
    tau = u_lqr(x,p);
else
    if p.method % noncollocated
        tau = u_noncollocated(x,p);
    else % collocated
        tau = u_collocated(x,p);
    end
end
end