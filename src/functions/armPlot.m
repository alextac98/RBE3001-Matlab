%%
% Arm Plotting Function using DH Parameters
% Developed by Alex Tacescu (https://alextac.com)
%%

% takes in DH Parameter matrix of any length

function armPlot(dh)
    cla;
    % TODO: Make this work for any length
    
    % Calculate dh transformation matricies
    t0_1 = tdh(dh(1, 1), dh(1, 2), dh(1, 3), dh(1, 4));
    t1_2 = tdh(dh(2, 1), dh(2, 2), dh(2, 3), dh(2, 4));
    t2_3 = tdh(dh(3, 1), dh(3, 2), dh(3, 3), dh(3, 4));
    
    % Caclulate each frame
    f0 = zeros(4, 4);
    f1 = t0_1;
    f2 = f1 * t1_2;
    f3 = f2 * t2_3;

    % Get poses in a nice format
    poses = zeros(4, 3);
    poses(1, :) = f0(1:3, 4);
    poses(2, :) = f1(1:3, 4);
    poses(3, :) = f2(1:3, 4);
    poses(4, :) = f3(1:3, 4);
    
    plot3(poses(:,1),poses(:,2),poses(:,3), ...
        '-o','LineWidth',2,'MarkerSize',6,'MarkerFaceColor',[0.5,0.5,0.5]);
    %hold on;
    grid on;
    
    xlim([-10 350]);
    ylim([-330 330]);
    zlim([0, 300]);

    %box on;
    ylabel('Y Axis (mm)');
    xlabel('X Axis (mm)');
    zlabel('Z Axis (mm)');
    title('Stick Arm');
    drawnow
    display(dh(:, 1));
end