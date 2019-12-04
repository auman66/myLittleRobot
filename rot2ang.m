
function[x, y, z] = rot2ang(Rphi)
% Extracts Euler angles from a rotation matrix
% [x, y, z]=ang2rot(R)
% Given a rotation matrix, rot2ang will decompose the matrix into the 
% original ZYX angles and output them. The function will also plot the 
% given matrix to show how it?s transformed from the (0, 0, 0) base frame. 
% The rotation is plotted.


zinv=(atan2(Rphi(2,1),Rphi(1,1)))/(pi/180);
yinv=(atan2(-Rphi(3,1),(sqrt((Rphi(3,2)^2)+(Rphi(3,3)^2)))))/(pi/180);
xinv=(atan2(Rphi(3,2),Rphi(3,3)))/(pi/180);

z=zinv;
y=yinv;
x=xinv;

p = [0 0 0]';
axis_start = p;

R = Rphi;

figure(1)
for i=1:3
    axis_end(:,i) = axis_start + R(:,i);
end
plot3(p(1), p(2), p(3), 'o');
grid on
hold on
axis equal
for i = 1:3
    h=plot3([axis_start(1) axis_end(1,i)],...
        [axis_start(2) axis_end(2,i)],...
        [axis_start(3) axis_end(3,i)]);
    if i==1
        h.Color='red';
    elseif i==2
        h.Color='green';
    else
        h.Color='blue';
    end
end
title('Rotated Matrix')
 
 
end