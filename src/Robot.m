
classdef Robot
    properties        
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
        function  shutdown(packet)
	    %Close the device
            packet.myHIDSimplePacketComs.disconnect();
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
        
    %% Basic Comms Functions
        % Perform a command cycle. This function will take in a command ID
        % and a list of 32 bit floating point numbers and pass them over the
        % HID interface to the device, it will take the response and parse
        % them back into a list of 32 bit floating point numbers as well
        function com = command(packet, idOfCommand, values)
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
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        function com = read(packet, idOfCommand)
                com= zeros(15, 1, 'single');
                try

                    % Default packet size for HID
                    intid = java.lang.Integer(idOfCommand);
                    %class(intid);
                    %class(idOfCommand);
                    %class(ds);
                    ret = 	packet.myHIDSimplePacketComs.readFloats(intid) ;
                    for i=1:length(com)
                       com(i)= ret(i).floatValue();
                    end
                    %class(com)
                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
        function  write(packet, idOfCommand, values)
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
                    packet.myHIDSimplePacketComs.writeFloats(intid,  ds,packet.pol);

                catch exception
                    getReport(exception)
                    disp('Command error, reading too fast');
                end
        end
    end
end
