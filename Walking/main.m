%% Passive Acrobot Walker
clear; clc; close all;

% Setup path
addpath(genpath('./'));

% Load dynamics and parameters
params = load('loadDynamics.mat');
params.num_steps = 10;
params.alpha = pi/50;
n_q = params.n_q;
t_traj = [];
x_traj = [];


%% Simulate (Animate after each impact)
q_init = [0; 0; 0.2255; -0.5767];
qdot_init = [0; 0; -1.123; 0.8414];
x_init = [q_init; qdot_init];  % [q; q_dot]
for i = 1:params.num_steps
    if i == 1
        x_init = x_init; % initial state remains unchanged
    else
        x_init = impact_reset_to_init(x_end,params);
    end
    tspan = 0:0.05:10;
    options = odeset('Event',@(t,x) Event_Impact(t,x,params));
    [t,x] = ode45(@(t,x) acro_pointmass_ode(t,x,params),tspan,x_init,options);
    t = t';
    x = x';
    
    % store traj
    t_traj = [t_traj, t];
    x_traj = [x_traj, x];
    
    % animate step
    animate_results(x,params);
    
    x_end = x(:,end);
    
end
%% Plot
% plot_results();


%% Extra Functions


function [value,isterminal,direction] = Event_Impact(t,x,p)
%     value(i) is the value of the ith event function.
%
%     isterminal(i) = 1 if the integration is to terminate at a zero of this
%         event function. Otherwise, it is 0.
%
%     direction(i) = 0 if all zeros are to be located (the default).
%         A value of +1 locates only zeros where the event function is increasing,
%         and -1 locates only zeros where the event function is decreasing.
isterminal = 1;
direction = 1;

q = x(1:p.n_q);
p_stance = p.p_stance_func(q);
p_swing = p.p_swing_func(q);

% value = 2*q(1) - 2*p.alpha + q(2)
value = -(p_swing(2)/p_swing(1)) - tan(p.alpha);
% value = t - p.t_impact;



end