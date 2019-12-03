clc;
clear;
mdl_planar3;
p3.links(1).I = eye(3);
p3.links(2).I = eye(3);
p3.links(3).I = eye(3);

STEPS = 20;

q0 = [pi/3, pi/2, pi/3];
q1 = [3*pi/2, pi/2, pi/3];
Td = p3.fkine(q0);
T1 = p3.fkine(q1);
xd = [transl(Td)', tr2rpy(Td)];
x1 = [transl(T1)', tr2rpy(T1)];

P = 100*eye(3);
D = 10*eye(3);

[t, q] = PD_interface(p3, xd, x1, 5.0, 50, P, D);
% q_dot0 = zeros(3,1);
% 
% q_des_array = p3.jtraj(Td, T1, STEPS, 'ikine', @p3.ikunc);
% % t_vec = linspace(0, 5, STEPS);
% 
% t_arr = [];
% q_arr = [];
% q_dot_arr = [];
% 
% q = q0;
% q_dot = q_dot0;
% for i = 1:length(q_des_array)
%     q_des = q_des_array(i, :);
%     [t_arr_curr, q_arr_curr, q_dot_arr_curr] = dyno_wrap(p3, 0.2632, 'PD', q, q_dot, q_des, P, D);
%     
%     t_arr = [t_arr; (i-1)*0.2632 + t_arr_curr];
%     q_arr = [q_arr; q_arr_curr];
%     q_dot_arr = [q_dot_arr; q_dot_arr_curr];
%     
%     
%     q = q_arr_curr(end, :);
%     q_dot = q_dot_arr_curr(end, :);
% 
% end