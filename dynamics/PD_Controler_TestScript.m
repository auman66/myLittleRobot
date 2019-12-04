clc;
clear;
% mdl_planar3;
% p3.links(1).I = eye(3);
% p3.links(2).I = eye(3);
% p3.links(3).I = eye(3);

mdl_stanford;
stanf.links(1).I = eye(3);
stanf.links(2).I = eye(3);
stanf.links(3).I = eye(3);
stanf.links(4).I = eye(3);
stanf.links(5).I = eye(3);
stanf.links(6).I = eye(3);

STEPS = 20;

q0 = [pi/2, pi/4, 1.0, pi/2, pi/3, pi/ 4];
q1 = [pi,   pi/4, 0.5, 0.4, pi/6, pi/ 4];
Td = stanf.fkine(q0);
T1 = stanf.fkine(q1);
xi = [transl(Td)', tr2rpy(Td)];
xf = [transl(T1)', tr2rpy(T1)];

P = 10;
D = 10;

[t1, q1] = PD_interface(stanf, xi, xf, 20.0, 50, P, D, "PD_Joint");

[t2, q2] = PD_interface(stanf, xi, xf, 20, 50, P, D, "PD_Oper");

