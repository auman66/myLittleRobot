
function[Rphi] = ang2rot(z,y,x)

%inputs for angles with respect to the baseframe
%  z=20;
%  y=20;
%  x=20;

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


% rotmZYZ=myrotmat(pi/3, 'z')*myrotmat(pi/2, 'y')*myrotmat(pi/6, 'z')
% rotmZYZ = eul2r(pi/3, pi/2, pi/6)
% p = [2.5 2.5 3.5]';
% axis_start = p;
% figure(2)
% for i=1:3
%     axis_end(:,i) = axis_start + rotmZYZ(:,i);
% end
% plot3(p(1), p(2), p(3), 'o');
% grid on
% hold on
% for i = 1:3
%     h=plot3([axis_start(1) axis_end(1,i)],...
%         [axis_start(2) axis_end(2,i)],...
%         [axis_start(3) axis_end(3,i)]);
%     if i==1
%         h.Color='red';
%     elseif i==2
%         h.Color='green';
%     else
%         h.Color='blue';
%     end
% end
% title('Part 2')
% R01=R; R02=rotmZYZ; 
% R21=(R01'*R02)'




 
 
end