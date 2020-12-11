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
% cam.ball_colors('pink')     = [115,  26,  62];
% cam.ball_colors('yellow')   = [183, 137,  36];
% cam.ball_colors('green')    = [ 56,  93,  14];
% cam.ball_colors('purple')   = [ 26,  23,  81];

%% Main Loop
try
    % Set up camera
    if cam.params == 0
        
        error("No camera parameters found!");
    end
    [cam.cam_pose, cam.ref_img] = cam.getCameraPose();
    
    isShutdown = false;
    
    i = 1;
    color_matrix = zeros(20, 3);
    while ~isShutdown
        % Get user input for next step
        in = input("Type `exit` to close, else will continue \n", 's');
        if strcmp(in, "exit")
            isShutdown = true;
            break;
        end
        
        robot.cmd_home();
        
        % Detect ball positions + color
        [pose, color] = cam.detectBestBall();
        
        if pose == 0
            disp("No more balls found in the task space");
            continue;
        end
        
        %pose(1, 4) = pose(1, 4) + 30;
        
        disp(color);
        
        disp(pose(1:3, 4)');
        
        robot.cmd_pick(pose(1:3, 4)');
        
        
        
        switch color
            case "yellow"
                robot.cmd_place(yellow_place);
            case "pink"
                robot.cmd_place(pink_place);
            case "purple"
                robot.cmd_place(purple_place);
            case "green"
                robot.cmd_place(green_place);
        end
        
    end

    
    
catch exception
    fprintf('\n ERROR!!! \n \n');
    disp(getReport(exception));
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
cam.shutdown()
