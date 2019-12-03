

vect=[1 1 1]';
%inputs for angles with respect to the baseframe
z=20;
y=40;
x=80;
btmrow=[0 0 0];

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
Rphi=Rz*Ry*Rx;
%transformation base -> frame from inputs

T=[Rphi vect;
    btmrow 1]

p = vect;
axis_start = p;

R_none=[0 0 0;
        0 0 0;
        0 0 0];
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

for i=1:3
    axis_end(:,i) = axis_start + R_none(:,i);
end
plot3(0, 0, 0, 'o');
grid on
hold on
axis equal
xlabel('X Translation');
ylabel('Y Translation');
zlabel('Z Translation');
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

title('Transformed Frame')


