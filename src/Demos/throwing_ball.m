
robot = Robot();
img = ImageProcessor();

disp("Please place the ball to pick up");
disp("Press any key to continue");
pause();

[coords, colors] = img.detect_balls();

robot.pickup_at_position(coords(1));

%Have robot go to a neutral position in line with the bin

% Set velocity stuff

% release once the joint is at some release point

%% IT WOULD BE REALLY COOL IF THIS WAS DONE WITH TWO QUINTIC SPLINES WHERE:
% One begins at 0 velocity and ends at x velocity
% the other begins at x velocity and ends at 0 velocity to make things
% smooth and not break the arm
% there is a call to robot.open_gripper() in between

