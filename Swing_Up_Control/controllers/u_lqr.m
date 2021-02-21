function u = u_lqr(x,p)
for i = 1:size(x,2)
    xi = x(:,i);
    xbar = [xi(1)-pi/2; xi(2); xi(3); xi(4)];
    u(i) = -p.K_lqr*xbar;
end

end