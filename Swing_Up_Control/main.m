%% Simulate Acrobot from Appendix (pg. 436)
clear; clc; close all;
addpath(genpath('./'));
params = load('loadDynamics');

% Choose between noncollocated (linearization about unactuated joint)
% or collocated ("" actuated joint)
prompt = "Enter (1) for noncollocated method or (0) for collocated method: ";
params.method = input(prompt);

%% Simulation
if params.method
    %% Non-collocated
    disp("Running simulation for noncollocation method (linearization about unacuated joint) ...");
    params.kp = 3; % 16
    params.kd = 1; % 5
    params.lqr = 0;
    p.Ts = 5; % switch time
    params.x_des = [pi/2; 0; 0; 0];
    
    x_init = [-pi/2; 0; 0; 0];
    tspan = 0:0.05:10;
    options = odeset('Events',@(t,x) myEventsFcn(t,x,p));
    [t1,x1] = ode45(@(t,x) acro_odefunc(t,x,params),tspan,x_init,options);
    x1 = x1';
    u1 = compute_ctrl(t1,x1,params);
    
    % Switch to lqr control
    params.lqr = 1;
    xinit = x1(:,end);
    xinit(2) = mod(xinit(2),2*pi);
    tspan = 0:0.05:5;
    [t2,x2] = ode45(@(t,x) acro_odefunc(t,x,params),tspan,xinit);
    t2 = t2 + t1(end);
    x2 = x2';
    u2 = compute_ctrl(t2,x2,params);  % Calculate Control Inputs that were used
    
else
    %% Collocated Method
    disp("Running simulation for collocation method (linearization about unacuated joint) ...");
    params.kp = 16; % 16
    params.kd = 5; % 5
    params.lqr = 0;
    params.Ts = 12; % switch time
    params.x_des = [pi/2; 0; 0; 0];
    params.alpha = 0.4;
    
    x_init = [0; 0; 0; 0];
    tspan = 0:0.05:15;
    options = odeset('Events',@(t,x) myEventsFcn(t,x,params));
    [t1,x1] = ode45(@(t,x) acro_odefunc(t,x,params),tspan,x_init,options);
    x1 = x1';
    u1 = compute_ctrl(t1,x1,params);
    
    % Switch to lqr control
    params.lqr = 1;
    xinit = x1(:,end);
    xinit(2) = mod(xinit(2),2*pi);
    tspan = 0:0.05:5;
    [t2,x2] = ode45(@(t,x) acro_odefunc(t,x,params),tspan,xinit);
    t2 = t2 + t1(end);
    x2 = x2';
    u2 = compute_ctrl(t2,x2,params);  % Calculate Control Inputs that were used
end

%% Plots
plotResults([t1; t2], [x1, x2],[u1, u2],params)

%% Animations
disp("Controller 1 Animation");
animateResults(x1,params);

disp("LQR Switching Controller Animation");
animateResults(x2,params);

disp("Simulation Complete");

%% ODE Event for Switching Controllers to LQR (time-based trial and error)
function [value,isterminal,direction] = myEventsFcn(t,x,p)
%     value(i) is the value of the ith event function.
%
%     isterminal(i) = 1 if the integration is to terminate at a zero of this
%         event function. Otherwise, it is 0.
%
%     direction(i) = 0 if all zeros are to be located (the default).
%         A value of +1 locates only zeros where the event function is increasing,
%         and -1 locates only zeros where the event function is decreasing.
isterminal = 1;
direction = 0;
value = t - p.Ts;
end


