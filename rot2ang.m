
function[z,y,x] = rot2ang(Rphi)

%input is a rotation matrix
%Rphi=[0.8830   -0.2115    0.4190;
     %0.3214    0.9230   -0.2115;
    %-0.3420    0.3214    0.8830];
%matrix above is for all 3 angles @ 20 degrees
    
    
%takes angles from rotation matrix using ZYX Euler angles

zinv=(atan2(Rphi(2,1),Rphi(1,1)))/(pi/180);
yinv=(atan2(-Rphi(3,1),(sqrt((Rphi(3,2)^2)+(Rphi(3,3)^2)))))/(pi/180);
xinv=(atan2(Rphi(3,2),Rphi(3,3)))/(pi/180);

z=zinv
y=yinv
x=xinv

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