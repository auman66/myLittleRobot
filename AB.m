BaseFrame=[1 0 0; 0 1 0; 0 0 1];

%inputs for angles with respect to the baseframe
a=0;
b=20;
g=0;
Pvect=[3 0 1]';

Arad=a*pi/180;
Brad=b*pi/180;
Grad=g*pi/180;
BtmRow=[0 0 0];

Rz=[cos(Arad) -sin(Arad) 0;
    sin(Arad) cos(Arad) 0;
    0 0 1];

Ry=[cos(Brad) 0 sin(Brad);
    0 1 0;
    -sin(Brad) 0 cos(Brad)];

Rx=[1 0 0;
    0 cos(Grad) -sin(Grad);
    0 sin(Grad) cos(Grad)];

%using ZYX premultiplication
%overall rotation matrix
Rphi=Rz*Ry*Rx;
%transformation base -> frame from inputs
Tbd=[Rphi Pvect;
     BtmRow 1]
 
 
 
 
  function=PlotFrame(Tbd)
  {
  figure(1)
  axis_start = [0 0 0];
  axis_end(:,1) = axis_start + [1 0 0];
  axis_end(:,2) = axis_start + [0 1 0];
  axis_end(:,3) = axis_start + [0 0 1];
  
  plot3(0,0,0, 'o');
  grid on;
  hold on;
  for i = 1.3
     h=plot3([axis_start(1) axis_end(1,i)],...
         [axis_start(2) axis_end(2,i)],...
         [axis_start(3) axis_end(3,i)]);
     if i==1
         h.Color='red';
         text(axis_end(1,i),...
         axis_end(2,i),...
         axis_end(3,i),'x_A','VerticalAlignment','bottom','HorizontalAlignment','right','fontsize',16);
     elseif i==2
         h.Color='green';
         text(axis_end(1,i),...
         axis_end(2,i),...
         axis_end(3,i),'y_A','VerticalAlignment','bottom','HorizontalAlignment','right','fontsize',16);
     else
         h.Color='blue';
         text(axis_end(i,1),...
         axis_end(3,i),'z_A','VerticalAlignment','bottom','HorizontalAlignment','right','fontsize',16);
     else
         h.Color='blue';
         text(axis_end(1,i),...
         axis_end(2,i),...
         axis_end(3,i),'z_A','VerticalAlignment','bottom','HorizontalAlignment','left','fontsize',16);
     end
  end
 
 
 
 
 }
 
 
 
 










