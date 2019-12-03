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

plot(t,q)