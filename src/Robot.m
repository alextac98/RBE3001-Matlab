%%
% RBE 3001 Arm Class in Matlab
% Developed by Alex Tacescu (https://alextac.com)
%%
classdef Robot
    properties
        % Flags
        DEBUG = false;
        STICKMODEL = false;
        
        %hidDevice;
        %hidService;
        myHIDSimplePacketComs
        pol
        
        % Package Server IDs
        gripper_id = 1962;
        setpoint_id = 1848;
        getpos_id = 1910;
        getvel_id = 1822;
        
        % Robot Position Status
        gripper_pos
        
        % Robot DH Parameters at home position
        dh_table = [
        %   theta    d     a   alpha
                0,  95,    0,  -pi/2;
            -pi/2,   0,  100,      0;
             pi/2,   0,  100,      0]
        
    end
    methods
    %% General Commands
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
            
            self.cmd_gripper(false);
            self.gripper_pos = false;
            
        end
        % The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
    %% Movement Commands
        % Command Gripper
        function cmd_gripper(self, open)
            packet = javaArray('java.lang.Byte', 1);
            if open
                packet(1) = java.lang.Byte(180);
                self.gripper_pos = true;
            else
                packet(1) = java.lang.Byte(0);
                self.gripper_pos = false;
            end
            try
                % Default packet size for HID
                intid = java.lang.Integer(self.gripper_id);
                self.myHIDSimplePacketComs.writeBytes(intid, packet, self.pol);
            catch exception
                getReport(exception)
                disp('Command error, reading too fast');
            end
        end
        % Command Joint-Space Trajectory
        function cmd_joint_traj(self, traj)
            % traj variable should be a matrix, where each row is a
            % different position in joint space
            packet = zeros(15, 1, 'single');
            for row = traj.'
                packet(1) = 1000;
                packet(2) = 0;
                packet(3) = row(1); % Joint 1 Position
                packet(4) = row(2); % Joint 2 Position
                packet(5) = row(3); % Joint 3 Position
                
                self.write(self.setpoint_id, packet);
                
                if self.STICKMODEL
                    self.live_stick_plot(packet(1)/1000);
                else
                    pause(packet(1)/1000);
                end
                
                if self.DEBUG
                    disp('Sent Packet:');
                    disp(packet);
                end
            end
        end
        
    %% Read Values Commands
        % Read current joint positions + position setpoints
        function pos = get_pos(self)
            % Output matrix has two columns:
                % 1st column: current position
                % 2nd column: current position setpoint
                
            returnPacket = self.read(self.getpos_id);
            
            pos = zeros(3, 2);
            pos(1, 1) = returnPacket(2); % Motor 1 Position
            pos(1, 2) = returnPacket(1); % Motor 1 Setpoint Position
            pos(2, 1) = returnPacket(4); % Motor 2 Position
            pos(2, 2) = returnPacket(3); % Motor 2 Setpoint Position
            pos(3, 1) = returnPacket(6); % Motor 3 Position
            pos(3, 2) = returnPacket(5); % Motor 3 Setpoint Position
            
            if self.DEBUG
                disp(pos);
            end
        end
        
        % Read current joint velocity + velocity setpoint
        function vel = get_vel(self)
            returnPacket = set.read(self.getvel_id);
            
            vel = returnPacket; % Not fully implemented
        end
     
    %% Graphing Functions
        % Live Stick Plot Function
        function live_stick_plot(self, time)
            t_inc = 0.5; % in seconds
            for t = 0:t_inc:time
                self.draw_stick_plot();
                while toc < t
                    % Wait until next timestamp is reached
                end
            end
        end
        % Draw Stick Plot Function
        function draw_stick_plot(self)
            tic;
            pos = self.get_pos();
            q = pos(:, 1);
            
            % Generate DH Table with joint angles
            dh = self.dh_table;
            dh(1, 1) = dh(1, 1) + q(1); % Joint 1
            dh(2, 1) = dh(2, 1) + q(2); % Joint 2
            dh(3, 1) = dh(3, 1) + q(3); % Joint 3
            
            armPlot(dh);
            toc
        end
        
    %% Basic Comms Functions
        % Perform a command cycle. This function will take in a command ID
        % and a list of 32 bit floating point numbers and pass them over the
        % HID interface to the device, it will take the response and parse
        % them back into a list of 32 bit floating point numbers as well
        function com = command(self, idOfCommand, values)
                com= zeros(15, 1, 'single');
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        function com = read(self, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    ret = 	self.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        function  write(self, idOfCommand, values)
                try
                    ds = javaArray('java.lang.Double',length(values));
                    for i=1:length(values)
                        ds(i)= java.lang.Double(values(i));
                    end
                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    self.myHIDSimplePacketComs.writeFloats(intid,  ds,self.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
    end
end
