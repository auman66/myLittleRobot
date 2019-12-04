
function[Rphi] = ang2rot(x, y, z)
% Create a rotation matrix based on Euler angle rotations
% R=ang2rot(x, y, z)
% Given the Z Y X angles in degrees, the ang2rot function will create a 
% rotation matrix corresponding to the given angles. The function will 
% output the rotation matrix and generate a plot showing the rotating 
% frame. The rotation is plotted. 


Zrad=z*pi/180;
Yrad=y*pi/180;
Xrad=x*pi/180;

Rz=[cos(Zrad) -sin(Zrad) 0;
    sin(Zrad) cos(Zrad) 0;
    0 0 1];

Ry=[cos(Yrad) 0 sin(Yrad);
    0 1 0;
    -sin(Yrad) 0 cos(Yrad)];

Rx=[1 0 0;
    0 cos(Xrad) -sin(Xrad);
    0 sin(Xrad) cos(Xrad)];

%using ZYX premultiplication
%overall rotation matrix
Rphi=Rz*Ry*Rx
%transformation base -> frame from inputs

Ginv=(atan2(Rphi(2,1),Rphi(1,1)))/(pi/180)
Binv=(atan2(-Rphi(3,1),(sqrt((Rphi(3,2)^2)+(Rphi(3,3)^2)))))/(pi/180)
Ainv=(atan2(Rphi(3,2),Rphi(3,3)))/(pi/180)


p = [0 0 0]';
axis_start = p;

R = Rphi

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