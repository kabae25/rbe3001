% Dynamic camera tracking: Modify your picking and sorting system to implement real-time object
% tracking (i.e. the MATLAB script should continuously grab frames from the camera and be able to track
% a certain object even if it is moved).

robot = Robot();
robot.open_gripper();

img = ImageProcessor();

disp("Add Ballz")
pause; 

while true
    % Find balls
    try
        [coords, colors] = img.detect_balls();
        if (size(coords))
            % Move the arm to the current Arm position with jacobian IK
            coords = coords(1,:);
            robot.jacobian_ik([coords, 100], 0.5);
        end
        pause(0.05); % pause for a little bit to not overload anything
    catch
        disp("No Ballzzz!")
        
    end
end