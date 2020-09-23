classdef Camera
    %CAMERA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        DEBUG = false;
        params;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.params = self.calibrate();
        end
        
        function params = calibrate(self)
            DEBUG = self.DEBUG;
            params = 0;
            try
                camcalib;
                params = cameraParams;
            catch exception
                getReport(exception);
                disp("No camerea calibration file found. Plese run camera calibration");
            end          
        end
        
        
    end
end

