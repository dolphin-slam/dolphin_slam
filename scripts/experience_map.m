%function experience_map()


em = importdata('experience_map.txt');
gt = importdata('ground_truth.txt');
dr = importdata('dead_reckoning.txt');

h = figure

a = axes;
set(a,'Zdir','reverse');
set(a,'Xdir','reverse');

xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');

hold on



plot3(em(:,1),em(:,2),em(:,3),'b-')
plot3(dr(:,1),dr(:,2),dr(:,3),'r-')
plot3(gt(:,1),gt(:,2),gt(:,3),'g-')
%Rotate3D(h, Center = [0,0,0], Axis = 
hold off

view(3)
%camroll(180)

grid on



%end

