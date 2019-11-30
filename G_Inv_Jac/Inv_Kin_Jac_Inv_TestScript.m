clear all
clc

L{1} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{2} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{3} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{4} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{5} = Link('theta', 0, 'a', 0.5, 'alpha', 0); 

R = SerialLink([L{1} L{2} L{3} L{4} L{5}]);

a1 = 0.5;
a2 = 0.5;
a3 = 0.5;
T = 0.001; %sec
t=0:T:4;

pd = [0.25*(1-cos(pi*t)); 0.25*(2+sin(pi*t))];
phid = sin(pi*t/24);
pd_dot = [0.25*(pi*sin(pi*t)); 0.25*(pi*cos(pi*t))];
phid_dot = pi/24*cos(pi*t/24);
m = [1, 1, 0, 0, 0, 1];

xd = [pd; phid];
xd_dot = [pd_dot; phid_dot];

q = inv_kin_JacInv(R, xd, xd_dot, m, T); 
