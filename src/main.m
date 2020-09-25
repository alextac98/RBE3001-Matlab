%%
% RBE 3001 Controller in Matlab
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
clear java;
format short

%% Flags
DEBUG = false;
STICKMODEL = false;
DEBUG_CAM = false;

%% Setup
vid = hex2dec('16c0');
pid = hex2dec('0486');

if DEBUG
    disp(vid);
    disp(pid);
end

javaaddpath ../lib/SimplePacketComsJavaFat-0.6.4.jar;
import edu.wpi.SimplePacketComs.*;
import edu.wpi.SimplePacketComs.device.*;
import edu.wpi.SimplePacketComs.phy.*;
import java.util.*;
import org.hid4java.*;
version -java;
myHIDSimplePacketComs=HIDfactory.get();
myHIDSimplePacketComs.setPid(pid);
myHIDSimplePacketComs.setVid(vid);
myHIDSimplePacketComs.connect();

robot = Robot(myHIDSimplePacketComs);
robot.DEBUG = DEBUG;
robot.STICKMODEL = STICKMODEL;

cam = Camera();
cam.DEBUG = DEBUG_CAM;

%% Place Poses per color
purple_place = [150, -50, 11];
green_place = [150, 50, 11];
pink_place = [75, -125, 11];
yellow_place = [75, 125, 11];

%% Set colors rgb values
cam.ball_colors('pink')     = [115,  26,  62];
cam.ball_colors('yellow')   = [183, 137,  36];
cam.ball_colors('green')    = [ 56,  93,  14];
cam.ball_colors('purple')   = [ 26,  23,  81];

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        error("No camera parameters found!");
    end
    cam.cam_pose = cam.getCameraPose();
    
    isShutdown = false;
    
    i = 1;
    color_matrix = zeros(20, 3);
    pause;
    while i < 100
        % Get user input for next step
        %in = input("Type `e` to close, else will continue", 's');
        %if strcmp(in, "e")
         %   isShutdown = true;
         %   break;
        %end
        
        % Detect ball positions
        [pose, color] = cam.detectBestBall();
        
        color_matrix(i, :) = color;
        i = i + 1;
    end
    
    mean = [mean(color_matrix(:, 1)), ...
            mean(color_matrix(:, 2)), ...
            mean(color_matrix(:, 3))]
        
    std =  [std(color_matrix(:, 1)), ...
            std(color_matrix(:, 2)), ...
            std(color_matrix(:, 3))]
    
    
    %robot.cmd_home();
    
    pick = [100, 25, 11];
    place = [150, -40, 11];
     
    %robot.cmd_pick(pick);
    %robot.cmd_place(yellow_place);
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
