% Wait for a ball (or multiple balls) to be placed onto the field
% 2. Once a ball is detected, pick it up and put it into its bin (you get to choose where exactly
% the bins are, but each color needs a unique bin)
% 3. Continue forever

robot = Robot();

img = ImageProcessor();
img.debug = 1;
while true
    try
        disp("Place balls on checkboard and press any key to continue...");
        pause; 
    
        [coords, colors] = img.detect_balls();
        
        while (size(colors)) % Repeat as long as there are balls left to collect
            robot.pick_up_ball(coords(1,:), colors(1));
            coords(1,:) = [];
            colors(1,:) = [];
        end

    catch
        disp("No Balls!")
    end
end 