%%
% Arm Plotting Function using DH Parameters
% Developed by Alex Tacescu (https://alextac.com)
%%

% takes in DH Parameter matrix of any length

function armPlot(dh)
clf
% Lenghts of linkages of arm, measurements in mm
linkage1 = 135;
linkage2 = 175;
linkage3 = 169.28;

waistAngle = -q(1);
armAngle = -q(2);
wristAngle = -q(3);

%we are using frames to reate the live plot of the arm as it seems to work,
%plot3 was upset
frame1 = tdh(waistAngle, linkage1, -90, 0);
frame2 = tdh(waistAngle, linkage1, -90, 0) * tdh(armAngle, 0, 0, linkage2);
frame3 = tdh(waistAngle, linkage1, -90, 0) * tdh(armAngle, 0, 0, linkage2) * tdh(wristAngle + 90, 0, 0, linkage3);

waistCoordinates = [frame1(1,4), frame1(2,4), frame1(3,4)];
armCoordinates = [frame2(1,4), frame2(2,4), frame2(3,4)];
wristCoordinates = [frame3(1,4), frame3(2,4), frame3(3,4)];

x1 = [0, waistCoordinates(1)];
y1 = [0, waistCoordinates(2)];
z1 = [0, waistCoordinates(3)];

x2 = [waistCoordinates(1), armCoordinates(1)];
y2 = [waistCoordinates(2), armCoordinates(2)];
z2 = [waistCoordinates(3), armCoordinates(3)];

x3 = [armCoordinates(1), wristCoordinates(1)];
y3 = [armCoordinates(2), wristCoordinates(2)];
z3 = [armCoordinates(3), wristCoordinates(3)];

plot3(0, 0, 0, 'o')
hold on


xlim([-10 350]);
ylim([-330 330]);
zlim([-50 300]);

line([0 x1], [0 -y1], [0 z1], 'Color', 'm','LineWidth', 2);
line([x1 x2], [-y1 -y2], [z1 z2], 'Color', 'm','LineWidth', 2);
line([x2 x3], [-y2 -y3], [z2 z3], 'Color', 'm','LineWidth', 2);

plot3(x1, -y1, z1, 'kp', 'MarkerFaceColor', [0 0 0], 'MarkerSize', 10)
plot3(x2, -y2, z2, 'kp', 'MarkerFaceColor', [0 0 0], 'MarkerSize', 10)
plot3(x3, -y3, z3, 'kp', 'MarkerFaceColor', [0 0 0], 'MarkerSize', 10)

grid on;
box on;
ylabel('Y Axis (mm)');
xlabel('X Axis (mm)');
zlabel('Z Axis (mm)');
title('Stick Arm');
end