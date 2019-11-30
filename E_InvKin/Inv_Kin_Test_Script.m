%Inverse Kinematics Test Script
%
%Meant to call inv_kin_pso() that will eventually get called by GUI.

clc;
clear all; 
close all; 




L{1} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{2} = Link('d', 0, 'a', 0.5, 'alpha', 0); 
L{3} = Link('d', 0, 'a', 0.5, 'alpha', 0); 

R = SerialLink([L{1} L{2} L{3}]);

%Use this to produce a Operational Space position to aim for
q_des = [-0.5, -0.5, 0];  

T_des = R.fkine([q_des]) 

m=[1 1 0 0 0 1]';
q0=[pi, -pi/2, -pi/2];  
%[sol_arr] = inv_kin_pso(R, T_des, q0, m);
[sol_arr] = inv_kin_rand_ikine(R, T_des, q0,m);


%This just animates through all the solutions, so I can see a visual
%difference
R.plot(repmat(sol_arr, [27, 1]))