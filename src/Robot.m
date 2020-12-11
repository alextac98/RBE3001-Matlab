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
        
        % Robot Default Positions (in task space)
        home_pos = [0, 118.3594,  33.2813];
         
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
        
        % Command Pick from surface
        % [pos] - task space pos where to pick up
        function cmd_pick(self, pos)
            % Open gripper
            self.cmd_gripper(true);
            % Create trajectory
            pos(1) = pos(1);
            pos(2) = pos(2) - 5;
            prepick_pos = pos;
            prepick_pos(3) = pos(3) + 30;
            traj = [prepick_pos; pos];
            % Move to pick up
            self.cmd_task_traj(traj);
            pause(1);
            % Close gripper
            self.cmd_gripper(false);
            % Move home
            self.cmd_home();
        end
        % Command Place
        % [pos] - task space pos where to drop off
        % Assumes already picked up an item
        function cmd_place(self, pos)
            % Create Trajectory
            int_pos = pos;
            int_pos(3) = pos(3) + 30;
            traj = [int_pos; pos];
            % Move to place
            self.cmd_task_traj(traj);
            % Open gripper
            self.cmd_gripper(true);
            % Move to intermediate place
            self.cmd_task_traj(int_pos);
            % Move home
            self.cmd_home();
            % Close gripper
            self.cmd_gripper(false);
        end
    
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
            pause(0.3);
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
        % Command Task-Space Trajectory
        function cmd_task_traj(self, traj)
            joint_traj = traj;
            
            for i = 1:size(traj, 1)
                joint_traj(i, :) = self.ikin(traj(i, :));
            end
            self.cmd_joint_traj(joint_traj);
        end
        
        function cmd_home(self)
           self.cmd_joint_traj([0, 0, 0]); 
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
        
        % Read current joint positions and return task position of end
        % effector
        function pos = get_task_pos(self)
           joint_pos = self.get_pos();
           pos = self.ikin(joint_pos(:, 1));
        end
        
        % Read current joint velocity + velocity setpoint
        function vel = get_vel(self)
            returnPacket = set.read(self.getvel_id);
            
            vel = returnPacket; % Not fully implemented
        end
        
    %% Calculation Functions
        % Calculate forward kinematics, inputs joint angles, outputs
        % position
        function t0_3 = fwkin(self, q)
            dh = self.dh_table;
            
            q1 = deg2rad(q(1));
            q2 = deg2rad(q(2));
            q3 = deg2rad(q(3));
            
            dh(1, 1) = dh(1, 1) + q1; % Joint 1
            dh(2, 1) = dh(2, 1) + q2; % Joint 2
            dh(3, 1) = dh(3, 1) + q3; % Joint 3
            
            t0_1 = tdh(dh(1, 1), dh(1, 2), dh(1, 3), dh(1, 4));
            t1_2 = tdh(dh(2, 1), dh(2, 2), dh(2, 3), dh(2, 4));
            t2_3 = tdh(dh(3, 1), dh(3, 2), dh(3, 3), dh(3, 4));
            
            t0_3 = t0_1 * t1_2 * t2_3;
    
        end
        
        % Calculate inverse kinematics, inputs desired pos, outputs joint
        % angles
        function q = ikin(self, p)
            px = round(p(1));
            py = round(p(2));
            pz = round(p(3));
            
            l1 = self.dh_table(1, 2);
            l2 = self.dh_table(2, 3);
            l3 = self.dh_table(3, 3);
            
            s = pz - l1;
            m = sqrt(px^2 + py^2);
            r = sqrt(m^2 + s^2);
            c = (l2^2 + r^2 - l3^2) / (2 * l2 * r);
            d = sqrt(1 - c^2);
            e = (l3^2 + l2^2 - r^2) / (2 * l3 * l2);
            f = sqrt(1 - e^2);
            alpha1 = atan2d(s, m);
            alpha2 = atan2d(d, c);
            alpha3 = atan2d(f, e);
            
            % calculating theta
            q = zeros(1, 3);
            q(1) = atan2d(py, px);
            q(2) = 90 - (alpha1 + alpha2);
            q(3) = 90 - alpha3;
            
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
            q = deg2rad(pos(:, 1));
            
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
