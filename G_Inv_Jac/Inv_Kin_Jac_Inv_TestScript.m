clear all;
clc;
close all;

L{1} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{2} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{3} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{4} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{5} = Link('theta', 0, 'a', 0.5, 'alpha', 0, 'prismatic'); 

R = SerialLink([L{1} L{2} L{3} L{4} L{5}]);

xd = [0.7, 0.2, 1, 0, 0, pi/2];

q = inv_kin_JacInv(R, xd); 
