classdef Camera
    %CAMERA Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        % Flags
        DEBUG = false;
        POSE_PLOT = false;  
        DEBUG_BALLDETECTION = false;
        
        base2checkerboard =  ...
            [ 0, 1, 0,   50; ...
              1, 0, 0, -100; ...
              0, 0, -1,   0; ...
              0, 0,  0,   1];
        
        % Image Processing 
        magnification = 50;
        min_blob_size = 3000;
        
        % Colors (as measured by Alex Tacescu)
        ball_colors =         ...
            [115,  26,  62; ...   % pink
             183, 137,  36; ...   % yellow
              56,  93,  14; ...   % green
              26,  23,  81];      % purple
         ball_color_keys = ...
             ["pink",   ...
              "yellow", ...
              "green",  ...
              "purple"];
        
        % Properties
        params;
        cam;
        cam_pose;
    end
    
    methods
        function self = Camera()
            % CAMERA Construct an instance of this class
            self.cam = webcam(); % Get camera object
            self.params = self.calibrate();
            
        end
        
        % Detect Balls + Report back positions
        function [pose, color] = detectBestBall(self)
            
            % Get an undistorted image
            [raw_img, new_origin] = self.getImg();
            
            % Segment image
            imgHSV = rgb2hsv(raw_img);
            saturation = imgHSV(:, :, 2);
            t = graythresh(saturation);
            imgBalls = (saturation > t);
            
            if self.DEBUG || self.DEBUG_BALLDETECTION
                figure; imshow(imgBalls, 'InitialMagnification', self.magnification);
                title('Segmented Balls');
            end
            
            % Find connected components.
            blobAnalysis = vision.BlobAnalysis('AreaOutputPort', true,...
                'CentroidOutputPort', true,...
                'BoundingBoxOutputPort', true,...
                'MinimumBlobArea', self.min_blob_size, ...
                'ExcludeBorderBlobs', true);
            
            [areas, centroids, boxes] = step(blobAnalysis, imgBalls);
            
            max_area = 0;
            pose = 0;
            for i = 1:size(areas)
                tmp_pose = self.cam2base(centroids(i, :) + new_origin);
                if abs(tmp_pose(2, 4)) > 125 || tmp_pose(1, 4) > 125
                    continue;
                end
                if areas(i) > max_area
                    best_centroid = centroids(i, :);
                    pose = tmp_pose;
                end
            end
            
            if pose == 0
                color = "";
                return;
            end

            % If in Debug mode or debugging object detection, draw graph
            if self.DEBUG || self.DEBUG_BALLDETECTION
                % Reduce the size of the image for display.
                scale = self.magnification / 100;
                imDetectedBalls = imresize(raw_img, scale);
                imDetectedBalls = insertObjectAnnotation(imDetectedBalls, ...
                    'rectangle', ...
                    scale * boxes, 'balls');
                figure; imshow(imDetectedBalls);
                title('Detected Coins');
            end
            
            % Get pose of ball in base frame
%             pose = self.cam2base(best_centroid + new_origin);
            
            % Get color of the ball
            color_rgb = impixel(raw_img, best_centroid(1), best_centroid(2));
            
            best_score = 10000;
            
            for i = 1:size(self.ball_color_keys, 2)
                score = mean(abs(color_rgb - self.ball_colors(i, :)));
                if score < best_score
                    color = self.ball_color_keys(i);
                    best_score = score;
                end
            end
            
        end
        
        function shutdown(self)
            clear self.cam;
        end
        
        function params = calibrate(self)
            DEBUG = self.DEBUG;
            params = 0;
            try
                disp("Clear surface of any items, then press any key to continue");
                pause;
                camcalib;
                params = cameraParams;
                disp("Camera calibration complete!");
            catch exception
                getReport(exception);
                disp("No camerea calibration file found. Plese run camera calibration");
            end          
        end
        
        % Get transformation from camera to checkerboard frame
        function pose = getCameraPose(self)
            % 1. Capture image from camera
            raw_img =  snapshot(self.cam);
            % 2. Undistort Image based on params
            [img, new_origin] = undistortImage(raw_img, self.params, 'OutputView', 'full');
            % 3. Detect checkerboard in the image
            [imagePoints, boardSize] = detectCheckerboardPoints(img);
            % 4. Adjust imagePoints so they are in the same frame of
            % reference as original image
            imagePoints = imagePoints + new_origin;
            % 5. Compute transformation
            [R, t] = extrinsics(imagePoints, self.params.WorldPoints, self.params);
            
            pose = [   R,    t';
                    0, 0, 0, 1];
                
            if self.POSE_PLOT
                axesPoints = worldToImage(self.params, R, t, [0 0 0; 0 50 0; 50 0 0]);
                
                x1 = [axesPoints(1, 1), axesPoints(2, 1)]';
                y1 = [axesPoints(1, 2), axesPoints(2, 2)]';
                
                img = insertText(img, [x1(2), y1(2)], 'Y Axis', 'TextColor', 'green', 'FontSize', 18);
                x2 = [axesPoints(1, 1), axesPoints(3, 1)]';
                y2 = [axesPoints(1, 2), axesPoints(3, 2)]';
                
                img = insertText(img, axesPoints(3, 1:2), 'X Axis', 'TextColor', 'red', 'FontSize', 18);
                
                imshow(img)
                title('Undistorted Image with checkerboard axes');
                
                line(x1, y1, 'linewidth', 5, 'Color', 'green');
                line(x2, y2, 'linewidth', 5, 'Color', 'red');
                
            end
                
        end
        
        % Transform camera to base frame of the robot
        function pose = cam2base(self, imagePoints)
            points = pointsToWorld(self.params, self.cam_pose(1:3, 1:3), self.cam_pose(1:3, 4), imagePoints);
            
            check_pose = ...
                [1, 0, 0, points(1); ...
                 0, 1, 0, points(2); ...
                 0, 0, 1,       -11; ...
                 0, 0, 0,         1];
             
             pose = self.base2checkerboard * check_pose;
        end
        
        % Take a picture and undistort image
        function [img, newOrigin] = getImg(self)
           imOrig = self.cam.snapshot();
           [img, newOrigin] = undistortImage(imOrig, self.params, 'OutputView', 'full');
        end
        
    end
end

