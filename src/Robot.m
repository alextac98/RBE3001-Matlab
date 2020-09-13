
classdef Robot
    properties
        % Flags
        DEBUG = false;
        
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
        gripper_pos = true;
        
    end
    methods
    %% General Commands
        % Create a packet processor for an HID device with USB PID 0x007
        function self = Robot(dev)
            self.myHIDSimplePacketComs=dev; 
            self.pol = java.lang.Boolean(false);
        end
        % The is a shutdown function to clear the HID hardware connection
        function  shutdown(self)
	    %Close the device
            self.myHIDSimplePacketComs.disconnect();
        end
        
    %% Movement Commands
        % Command Gripper
        function cmd_gripper(self, open)
            if open
                self.write(self.gripper_id, 0);
                self.gripper_pos = true;
            else
                self.write(self.gripper_id, 180);
                self.gripper_pos = false;
            end
           self.read(self.gripper_id);
        end
        % Command Joint-Space Trajectory
        function cmd_joint_traj(self, traj)
            % traj variable should be a matrix, where each row is a
            % different position in joint space
            packet = zeros(15, 1, 'single');
            for row = traj.'
                packet(1) = 100;
                packet(2) = 0;
                packet(3) = row(1); % Joint 1 Position
                packet(4) = row(2); % Joint 2 Position
                packet(5) = row(3); % Joint 3 Position
                
                self.write(self.setpoint_id, packet);
                
                returnPacket = self.read(self.getpos_id);
                
                if self.DEBUG
                    disp('Sent Packet:');
                    disp(packet);
                    disp('Received Packet:');
                    disp(returnPacket);
                end
                
                pause(0.5);
            end
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
