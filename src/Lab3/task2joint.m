% Create a robot object
robot = Robot();
travel_time = 3;  % CHOOSE SOMETHING REASONABLE

% Define three [x, y, z, alpha] positions for your end effector that all
% lie upon the x-z plane of your robot

% For each leg of your triangle, calculate the TASK SPACE trajectory
% between each vetex. Remember, to calculate a task space trajectory 
% you will need to create individual trajectories for x, y, z, and alpha.

% Move your robot to the first vertex


% Create a for loop that sends your robot to each vertex, recording the
% JOINT-SPACE position the entire time
for i=1:3
    tic;
    while toc < travel_time
        % Evaluate your trajectory using eval_traj

        % Pass your target x, y, z, alpha into IK to get joint values
        
        % Move your robot using your method of choice

    end
end

% Loop through all collected data and convert it from joint-space to task
% space using your fk3001 function

% Plot the trajectory of your robot in x-y-z space using scatter3
% https://www.mathworks.com/help/matlab/ref/scatter3.html


% Plot the trajectory of your robot in theta2-theta3-theta-4 space using
% scatter3
