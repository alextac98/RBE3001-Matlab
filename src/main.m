%%
% RBE 3001 Controller in Matlab
% Developed by Alex Tacescu (alextac.com)
%%
clc;
clear;
clear java;
clear classes;

%% Flags
DEBUG = false;

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

%% Main Control
try
    traj = [0, 0, 0;
            40, 40, 40;
            20, 10, 0;
            0, 0, 0];
    robot.cmd_joint_traj(traj);
catch exception
    getReport(exception)
    disp('Exited on error, clean shutdown');
end

%% Shutdown Procedure
robot.shutdown()
