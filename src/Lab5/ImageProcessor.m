classdef ImageProcessor < handle
    properties 
        camera;  % The Camera object for this image processor
        debug;
        checkerMask;
    end

    methods
        function self = ImageProcessor(nvargs)
            arguments
                nvargs.debug logical = false;
            end

            self.camera = Camera();  % Instantiate a Camera object
            self.checkerMask = self.generate_static_mask(); % Grab the mask while the board is empty
        end
        
        %% GENERATE_STATIC_MASK produces a binary mask that leaves only
            %the checkerboard and surrounding area visible
            % Inputs: 
            %   margin (optional): a number (what units?) indicating how
            %                      far from the edge of the checkerboard 
            %                      to keep in the mask
            % Outputs:
            %   mask: a binary mask that blacks out everything except a
            %         region of interest around the checkerboard

            % If self.debug == true, this function should capture a
            % picture, apply the mask, and display it
        function mask = generate_static_mask(self, nvargs)
            arguments
                self ImageProcessor;
                nvargs.margin double = 100;
            end
            

            % First Get the Image
            frame = self.camera.getImage();

            % Then get the edges of the checkerboard
            [imagePoints, boardSize] = detectCheckerboardPoints(frame, 'PartialDetections', false);
            
            % Use the Four farthest points in the corners
            x = [imagePoints(1, 1)-40 imagePoints(4, 1)-110 imagePoints(40, 1)+100 imagePoints(37, 1)+40];
            y = [imagePoints(1, 2)-40 imagePoints(4, 2)+60 imagePoints(40, 2)+50 imagePoints(37, 2)-40];

            %x = [imagePoints(1, 1)-nvargs.margin imagePoints(4, 1)-nvargs.margin imagePoints(40, 1)+nvargs.margin imagePoints(37, 1)+nvargs.margin];
            %y = [imagePoints(1, 2)-nvargs.margin imagePoints(4, 2)+nvargs.margin imagePoints(40, 2)+nvargs.margin imagePoints(37, 2)-nvargs.margin];


            % Then Mask off the area + the margin & return
            [height, width, channels] = size(frame);
            
            mask = ~poly2mask(x, y, height, width);
            
            %% This needs work to be up to spec of debug flag
            if self.debug
                frame(repmat(mask, [1, 1, channels])) = 0;
                imshow(frame);
            end
            
        end

        function p_robot = image_to_robot(self, uvpos)
            %IMAGE_TO_ROBOT transforms a point on the image to the
            %corresponding point in the frame of the robot
            % Inputs:
            %   uvpos: a [1x2] matrix representing an image (u, v) 
            %          coordinate
            % Outputs:
            %   p_robot: a [1x2] matrix representing the transformation of
            %            the input uvpos into the robot's base frame
            
            % Image to Checker
            checker_pos = pointsToWorld(self.camera.getCameraIntrinsics, self.camera.getRotationMatrix, self.camera.getTranslationVector, uvpos);
        
            % Checker to Robot
            checker_to_robot_rot = [0, 1, 0; % representation of robot frame in checker frame
                                    1, 0, 0;
                                    0, 0,-1];
            
            checker_to_robot_trnsl = [eye(2), transpose([-115 80]);
                                         0,      0,     1]; % this makes the translational offset matrix in 3d

            checker_pos = [eye(2), transpose(checker_pos); % this is the position on the checkerboard
                                0,       0,          1];

            robot_frame = checker_to_robot_rot * checker_pos;
            robot_pos = robot_frame * checker_to_robot_trnsl;
            p_robot = [robot_pos(1, 3) robot_pos(2, 3)];
        end
        
        %DETECT_CENTROIDS detects the centroids of binary blobs of
            %large enough size
            % Iputs: 
            %   Image: an image of the environment that has already been
            %          masked for the environment and to isolate a single 
            %          color
            %   min_size (optional): the minimum size of a blob to consider
            %                        a ball
            % Outputs: 
            %   colors: a [1xn] matrix of strings indicating the color of
            %           the ball at each detected centroid
            %   uv_centroids: a [nx2] matrix of coordinates of valid
            %                 centroids in image coordinates
            
            % If self.debug == true, this function should display the image
            % that was passed to it with colored circles on it marking the
            % balls

            % If self.debug == true, this function should display the image
            % that was passed to it with each color mask applied
            % individually (4 figures total for this part).
        function [uv_centroids, colors] = detect_centroids(self, image, nvargs)
            arguments
                self ImageProcessor;
                image uint8;
                nvargs.min_size double = 60; % was 60 % chooose a value
            end
            
            [height, width, channels] = size(image);            
            image(repmat(self.checkerMask, [1, 1, channels])) = 0;

            % Mask out the colors
            green_only = greenMask(image);
            yellow_only = yellowMask(image);
            orange_only = orangeMask(image);
            red_only = redMask(image);
            
            % Clean the Images
            green_clean = bwareaopen(green_only, nvargs.min_size); % 10 is a good number found through experimentation
            yellow_clean = bwareaopen(yellow_only, nvargs.min_size); % 10 is a good number found through experimentation
            orange_clean = bwareaopen(orange_only, nvargs.min_size); % 10 is a good number found through experimentation
            red_clean = bwareaopen(red_only, nvargs.min_size); % 10 is a good number found through experimentation

            % Find the Centroids
            [gLabeledImage gNumberOfObjects] = bwlabel(green_clean, 8);
            green_blobs = regionprops(gLabeledImage, "Centroid");
            
            [yLabeledImage yNumberOfObjects] = bwlabel(yellow_clean, 8);
            yellow_blobs = regionprops(yLabeledImage, "Centroid");

            [oLabeledImage oNumberOfObjects] = bwlabel(orange_clean, 8);
            orange_blobs = regionprops(oLabeledImage, "Centroid");

            [rLabeledImage rNumberOfObjects] = bwlabel(red_clean, 8);
            red_blobs = regionprops(rLabeledImage, "Centroid");

            colors = [];
            uv_centroids = [];

            % Compile function outputs
            uv_centroids = cat(1, uv_centroids, green_blobs.Centroid);
            for i=1:size(green_blobs, 1)
                colors = cat(1, colors, "green");
            end

            uv_centroids = cat(1, uv_centroids, yellow_blobs.Centroid);
            for i=1:size(yellow_blobs, 1)
                colors = cat(1, colors, "yellow");
            end
            uv_centroids = cat(1, uv_centroids, orange_blobs.Centroid);
            for i=1:size(orange_blobs, 1)
                colors = cat(1, colors, "orange");
            end
            uv_centroids = cat(1, uv_centroids, red_blobs.Centroid)
            for i=1:size(red_blobs, 1)
                colors = cat(1, colors, "red");
            end

            
            if self.debug
                % Show the masked area
                imshow(image);
                % Plot the cleaned green mask with coordinates
                figure
                subplot(2, 2, 1)
                imshow(green_clean);
                hold on;
                title("Cleaned Green Threshold");
                for i = 1:numel(green_blobs)
                centroid = green_blobs(i).Centroid;
                % Display the coordinate as a formatted string near the centroid
                text(centroid(1), centroid(2), sprintf('(%0.1f, %0.1f)', centroid(1), centroid(2)),'Color', 'red', 'FontSize', 10, 'HorizontalAlignment', 'center');
                end
                hold off;

                
                subplot(2, 2, 2)
                imshow(yellow_clean);
                hold on;
                title("Cleaned Yellow Threshold");
                for i = 1:numel(yellow_blobs)
                centroid = yellow_blobs(i).Centroid;
                text(centroid(1), centroid(2), sprintf('(%0.1f, %0.1f)', centroid(1), centroid(2)), 'Color', 'red', 'FontSize', 10, 'HorizontalAlignment', 'center');
                end
                hold off;

                
                subplot(2, 2, 3)
                imshow(orange_clean);
                hold on;
                title("Cleaned Orange Threshold");
                for i = 1:numel(orange_blobs)
                centroid = orange_blobs(i).Centroid;
                text(centroid(1), centroid(2), sprintf('(%0.1f, %0.1f)', centroid(1), centroid(2)),  'Color', 'red', 'FontSize', 10, 'HorizontalAlignment', 'center');
                end
                hold off;

                
                subplot(2, 2, 4)
                imshow(red_clean);
                hold on;
                title("Cleaned Red Threshold");
                for i = 1:numel(red_blobs)
                    centroid = red_blobs(i).Centroid;
                    text(centroid(1), centroid(2), sprintf('(%0.1f, %0.1f)', centroid(1), centroid(2)),  'Color', 'red', 'FontSize', 10, 'HorizontalAlignment', 'center');
                 end
                hold off;
            end
        end
        
        %CORRECT_CENTROIDS transforms image coordinate centroids into
            %task-space coordinates for the ball
            % Inputs: 
            %   centroids: a [nx2] array of centroids in image coordinates
            %   ball_z (optional): how high the center of the ball is in
            %                      millimeters
        function ts_centroids = correct_centroids(self, centroids, nvargs)
            arguments
                self ImageProcessor;
                centroids double;
                nvargs.ball_z = 11.5; % was 10 
            end
            
            for i=1:(size(centroids))
                r = nvargs.ball_z; % radius of the sphere
                position_on_checkerboard = self.image_to_robot(centroids(i, :));

               
                projection_angle = -atan2d(position_on_checkerboard(2), position_on_checkerboard(1));
                
                if abs(projection_angle) > 25
                        ts_centroids(i, :) = [position_on_checkerboard(1)+3*r*cosd(projection_angle)...
                                      position_on_checkerboard(2)+1*r*sind(projection_angle)];
                else
                           ts_centroids(i, :) = [position_on_checkerboard(1)+1.2*r*cosd(projection_angle)...
                                      position_on_checkerboard(2)+ 1.2*r*sind(projection_angle)];
                end
            end

        end

        % Write a function that acquires an image, returns the
        % coordinates of the balls in the task space of the robot. Satisfy
        % the following requiremetns.
        
        %DETECT_BALLS finds the task space coordinates of all balls on the
        %checkerboard
        % Outputs:
        %   ts_coords: the task space coordinates of all balls in the
        %              workspace
        %   colors:    array of each ball color at that coordinate
        function [ts_coords, colors] = detect_balls(self)
            arguments
                self ImageProcessor;
            end
        
            % Aquire Image
            frame = self.camera.getImage();

            % Get the positions & colors of the centroids 
            [uv_positions, color] = self.detect_centroids(frame);
            colors = color;

            ts_coords = self.correct_centroids(uv_positions);
        end
    end
end
