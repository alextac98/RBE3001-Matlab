%%
% RBE 3001 Controller in Matlab
% Developed by Alex Tacescu (https://alextac.com)
%%
clc;
clear;
clear java;
clear classes;
format short

%% Flags
DEBUG = false;
STICKMODEL = true;

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

%% Main Control
try
    
    traj = [0, 0, - 90;
            40, 40, 40;
            0, 10, 0;
            0, 0, 0];
        
     for i = 1:size(traj,1)
         q = traj(i, :);
         p = robot.fwkin(q);
         p(1:3, 4)
         disp(robot.ikin(p(1:3, 4)));
     end
        
        
     % robot.cmd_joint_traj(traj);
        
    % pause(1);
    
%     robot.cmd_gripper(false);
        
    % robot.get_pos()
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
