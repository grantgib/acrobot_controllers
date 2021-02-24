function [value,isterminal,direction] = impact_event(t,x,p)
%     value(i) is the value of the ith event function.
%
%     isterminal(i) = 1 if the integration is to terminate at a zero of this
%         event function. Otherwise, it is 0.
%
%     direction(i) = 0 if all zeros are to be located (the default).
%         A value of +1 locates only zeros where the event function is increasing,
%         and -1 locates only zeros where the event function is decreasing.
isterminal = 1;
direction = -1;
qst = x(3);
qsw = x(4);
value = qst + qsw + 2*p.alpha;
end