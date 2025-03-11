% (c) 2023 Robotics Engineering Department, WPI
% Skeleton Robot class for OpenManipulator-X Robot for RBE 3001

classdef Robot < OM_X_arm
    % Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    % Hopefully, you should only need what's in this class to accomplish everything.
    % But feel free to poke around!
    properties
        mDim; % Stores the robot link dimentions (mm)
        mOtherDim; % Stores extraneous second link dimensions (mm)
        mPreviouslySavedPosition;
    end
    methods
        % Creates constants and connects via serial
        % Super class constructor called implicitly
        % Add startup functionality here
        function self = Robot()
            % Change robot to position mode with torque enabled by default
            % Feel free to change this as desired
            self.writeMode('p');
            self.writeMotorState(true);
            self.writeTime(0.55); % Write travel time

            % setup link dimentions for the robot (mm)
            self.mDim = [96.326, sqrt(128^2 + 24^2), 124, 133.4]; % (mm) Represents L1 + L0, L2, L3, L4
            
            % Open the gripper
            self.close_gripper();

            % Move to start position
            self.interpolate_jp([90, zeros(1,3)], 3)
            pause(3)
            self.mPreviouslySavedPosition = self.getTaskSpacePos();
        end

        % Sends the joints to the desired angles
        % goals [1x4 double] - angles (degrees) for each of the joints to go to
        function writeJoints(self, goals)
            if checkSafe(goals)
                goals = mod(round(goals .* DX_XM430_W350.TICKS_PER_DEG + DX_XM430_W350.TICK_POS_OFFSET), DX_XM430_W350.TICKS_PER_ROT);
                self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.GOAL_POSITION, goals);
            end
        end

        % Creates a time based profile (trapezoidal) based on the desired times
        % This will cause writePosition to take the desired number of
        % seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
        % time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
        % acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
        % acc_time is an optional parameter. It defaults to time/3.
      
        function writeTime(self, time, acc_time)
            if (~exist("acc_time", "var"))
                acc_time = time / 3;
            end

            time_ms = time * DX_XM430_W350.MS_PER_S;
            acc_time_ms = acc_time * DX_XM430_W350.MS_PER_S;

            self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC, acc_time_ms);
            self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL, time_ms);
        end
        
        % Sets the gripper to be open or closed
        % Feel free to change values for open and closed positions as desired (they are in degrees)
        % open [boolean] - true to set the gripper to open, false to close
        function writeGripper(self, open)
            if open
                self.gripper.writePosition(-35);
            else
                self.gripper.writePosition(55);
            end
        end

        % Sets position holding for the joints on or off
        % enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
        function writeMotorState(self, enable)
            self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable);
        end

        % Supplies the joints with the desired currents
        % currents [1x4 double] - currents (mA) for each of the joints to be supplied
        function writeCurrents(self, currents)
            currentInTicks = round(currents .* DX_XM430_W350.TICKS_PER_mA);
            self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.GOAL_CURRENT, currentInTicks);
        end

        % Change the operating mode for all joints:
        % https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
        % mode [string] - new operating mode for all joints
        % "current": Current Control Mode (writeCurrent)
        % "velocity": Velocity Control Mode (writeVelocity)
        % "position": Position Control Mode (writePosition)
        % Other provided but not relevant/useful modes:
        % "ext position": Extended Position Control Mode
        % "curr position": Current-based Position Control Mode
        % "pwm voltage": PWM Control Mode
        function writeMode(self, mode)
            switch mode
                case {'current', 'c'} 
                    writeMode = DX_XM430_W350.CURR_CNTR_MD;
                case {'velocity', 'v'}
                    writeMode = DX_XM430_W350.VEL_CNTR_MD;
                case {'position', 'p'}
                    writeMode = DX_XM430_W350.POS_CNTR_MD;
                case {'ext position', 'ep'} % Not useful normally
                    writeMode = DX_XM430_W350.EXT_POS_CNTR_MD;
                case {'curr position', 'cp'} % Not useful normally
                    writeMode = DX_XM430_W350.CURR_POS_CNTR_MD;
                case {'pwm voltage', 'pwm'} % Not useful normally
                    writeMode = DX_XM430_W350.PWM_CNTR_MD;
                otherwise
                    error("setOperatingMode input cannot be '%s'. See implementation in DX_XM430_W350. class.", mode)
            end

            lastVelTimes = self.bulkReadWrite(DX_XM430_W350.PROF_VEL_LEN, DX_XM430_W350.PROF_VEL);
            lastAccTimes = self.bulkReadWrite(DX_XM430_W350.PROF_ACC_LEN, DX_XM430_W350.PROF_ACC);

            self.writeMotorState(false);
            self.bulkReadWrite(DX_XM430_W350.OPR_MODE_LEN, DX_XM430_W350.OPR_MODE, writeMode);
            self.writeTime(lastVelTimes(1) / 1000, lastAccTimes(1) / 1000);
            self.writeMotorState(true);
        end

        % Gets the current joint positions, velocities, and currents
        % readings [3x4 double] - The joints' positions, velocities,
        % and efforts (deg, deg/s, mA)
        function readings = getJointsReadings(self)
            readings = zeros(3,4);
            
            readings(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            readings(2, :) = self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL;
            readings(3, :) = self.bulkReadWrite(DX_XM430_W350.CURR_LEN, DX_XM430_W350.CURR_CURRENT) ./ DX_XM430_W350.TICKS_PER_mA;
        end

        % Sends the joints at the desired velocites
        % vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
        function writeVelocities(self, vels)
            vels = round(vels .* DX_XM430_W350.TICKS_PER_ANGVEL);
            self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.GOAL_VELOCITY, vels);
        end
    
        %% LAB 

        function servo_jp(self, q)
        %SERVO_JP Send robot to a joint configuration
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
            
            self.writeTime(2); % Write travel time
            self.writeMotorState(true); % Write position mode
            self.writeJoints(q);
            pause(2);

        end
        
        function interpolate_jp(self, q, time)
        %INTERPOLATE_JP Send robot to a joint configuration over a period of time
        % Inputs:
        %    q: a [1x4] vector containing the target joint positions
        %    t: a scalar that tells how long to take to travel to the new position
        %       in milliseconds
           self.writeTime(time); % converts time to seconds for writetime so we dont send arm at mach 5
           %self.writeMotorState(true);
           self.writeJoints(q)

        end

        function q_curr = measure_js(self, GETPOS, GETVEL)
        %MEASURED_JS Get the current position and velocity of the robot
        % Inputs:
        %    GETPOS: a boolean indicating whether or not to retrieve joint
        %            positions
        %    GETVEL: a boolean indicating whether or not to retrieve joint
        %            velocities
        % Outputs:
        %    q_curr: a [2x4] matrix whose top row contains joint positions (0s if
        %            GETPOS is false), and whose bottom row contains joint 
        %            velocities
        
            q_curr = zeros(2,4);

            if GETPOS
                q_curr(1, :) = (self.bulkReadWrite(DX_XM430_W350.POS_LEN, DX_XM430_W350.CURR_POSITION) - DX_XM430_W350.TICK_POS_OFFSET) ./ DX_XM430_W350.TICKS_PER_DEG;
            end

            if GETVEL
                q_curr(2, :) = (self.bulkReadWrite(DX_XM430_W350.VEL_LEN, DX_XM430_W350.CURR_VELOCITY) ./ DX_XM430_W350.TICKS_PER_ANGVEL);
            end

          
        end
        %% LAB 2

        % dh2mat takes in a row of the dh table and outputs the homogenous transformation matrix
        % Where frame = [theta_i d_i a_i alhpa_i]
        function dhtable = dh2mat(self, frame)
            dhtable = [
                cosd(frame(1))  -sind(frame(1))*cosd(frame(4))  sind(frame(1))*sind(frame(4))   frame(3)*cosd(frame(1));
                sind(frame(1))  cosd(frame(1))*cosd(frame(4))   -cosd(frame(1))*sind(frame(4))  frame(3)*sind(frame(1));
                0               sind(frame(4))                  cosd(frame(4))                  frame(2);
                0               0                               0                               1
            ];

        end

        % dh2mat takes in the entire DH table and outputs the forward
        % kinematics transformation matrix
        function fkmat = dh2fk(self,dhtable)
            s = size(dhtable, 1); % get the number of columns in the dh table
            fkmat = eye(4);
            for joint = 1:s % iterate through all joints in the dh table
                fkmat = fkmat * self.dh2mat(dhtable(joint, :));
            end
        end    
        
        % outputs a 1x4 vextor x,y,z, alpha
        function V = fk_3001(self, theta1, theta2, theta3, theta4)
            V = generate_fast_fk(theta1, theta2, theta3, theta4);
        end

        %% BEGIN LAB 3 CODE

        % jointAngles takes in the position and orientation of the end
        % effector
        % location is a 4x1 matrix of x,y,z,alpha
        function jointAngles = ik_3001(self, location)
            alpha = location(4);
            offset = atan2d(24, 128);

            % Assign L lengths from arm dimensions
            for length = 1:4
                L(length) = self.mDim(length);
            end

            % Valid End Effector Pose checks
            Ra = sqrt(location(1).^2 + location(2).^2 + location(3).^2);
            if (Ra > (L(2) + L(3) + L(4))) % Check if the desired pose is farther than outstretched arm
                ErrID = 'ik_3001:Invalid';
                msg = 'Arm too short!';
                baseException = MException(ErrID, msg);
                throw(baseException);
            else
                %mustBePositive(location(1)) % check if x position is reachable (non-negative)
                %mustBePositive(location(4)) % check if z position is reachable (non-negative)
            end
            
            % https://www.desmos.com/calculator/qrx0droran
            theta1 = atan2d(location(2), location(1));
            F4x = location(1) - (L(4)*cosd(alpha))*(cosd(theta1));
            F4y = location(2) - (L(4)*cosd(alpha))*(sind(theta1));
            F4z = location(3) + L(4)*sind(alpha);

            % Arm-Plane
            Rv = F4z - L(1);
            Rh = sqrt(F4x.^2 + F4y.^2); % this line differs in implementation
            R = sqrt(Rh.^2 + Rv.^2);
            Z = atan2d(Rv, Rh); % zeta
            
            % Solving Law of cosine angles
            E_int = ((R.^2 - L(2).^2 - L(3).^2)/(-2*L(2)*L(3)));
            E = atan2d(sqrt(1 - E_int.^2), E_int);
            B_int = ((L(3).^2 - L(2).^2 - R.^2)/(-2*L(2)*R));
            B = atan2d(sqrt(1 - B_int.^2), B_int);
            
            % Solving Angles
            theta2 = 90 - B - Z - offset;
            theta3 = 90 - E + offset;
            theta4 = alpha - (180 - E - (B + Z));

            jointAngles = [theta1, theta2, theta3, theta4];
        end

        %% LAB 4 CODE
        
        %JACOB3001 Calculates the jacobian of the OMX arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            % Outputs:
            %   j: a [6x4] jacobian matrix of the robot in the given pos
        function j = jacob3001(self, qpos) 
            j = Jacobian(qpos(1), qpos(2), qpos(3), qpos(4));
            
        end

        % DK3001 Calculates the forward velocity kinematics of the OMX
            %arm
            % Inputs:
            %   qpos: a [1x4] matrix composed of joint positions
            %   qvel: a [1x4] matrix composed of joint angular velocities
            % Outputs:
            %   vs: a [6x1] matrix representing the linear and angular 
            %       velocity of the end effector in the base frame of the 
            %       robot
        function vs = dk3001(self, qpos, qvel)
            j_pos = self.jacob3001(qpos);
            vs = j_pos * qvel;
        end
         
 % Jacobian IK Calculates the inverse kinematics using Newton
            %Raphson Method
            % Inputs:
            %   Location: a [1x3] matrix representing coordinates in Task
            %   space
            %   Theshold: a scalar representing an acceptable tolerance
            %   between the approached position and target position.
            function jacobian_ik(self, location, threshold)
            joint_states = self.measure_js(true, false); % Get joint positions
            q_guess = joint_states(1,:); % take only the positions

            %q_guess = [0; 0; 0; 0]; % Initial starting point
            %self.interpolate_jp(q_guess, 2)
            i = 1;
            %pause(2.5)
            
            while true
                fk = self.fk_3001(q_guess(1), q_guess(2), q_guess(3), q_guess(4));
                fk(:, 4)
                q_guess
                
                diff = location - transpose(fk(1:3, 4));
                if(abs(diff) < threshold)
                    break
                end 

                J = self.jacob3001(transpose(q_guess));
                invJ = pinv(J(1:3, :));
                increment = invJ * transpose(diff)
                q_guess = q_guess + transpose(increment);
                self.interpolate_jp(q_guess, 0.55);
                % fprintf('Iteration Count: %d\n',i);
                i = i + 1;

            end
        end

        % the lab document
            % atSingularity detects if the arm is approaching a singularity
            % Inputs:
            %   threshold   : value of determinant to stop at
            %   qpos        : state variables to take the jacobian of
            %   handle      : boolean flag to throw exception
            %                 1 = throw / 0 = do nothing
            % Outputs:
            %   arrived     : boolean flag if determinant of the jacobian
            %                 is below provided threshold
        function arrived = atSingularity(self, threshold, qpos, throw_exception)

            j_pos = self.jacob3001(qpos);
            pos_deter = det(j_pos(1:3, 1:3));
            arrived = det(pos_deter) < threshold;

            if arrived && throw_exception
                ErrID = 'atSingularity:Singularity';
                msg = 'Arm is at a singularity, stopping!';
                baseException = MException(ErrID, msg);
                throw(baseException);
            end
        end

        %% Lab 5

        %BLOCKING_TS_MOVE moves the robot in a straight line in task space 
        %to the target position before exiting
        % Inputs:
        %   pos: a [1x4] matrix representing a target x, y, z, alpha
        %   time (optional): an integer representing desired travel time
        %         in seconds. Default time: 2 seconds.
        %   mode (optional): a string "cubic" or "quintic" indicating what 
        %                    type of trajectory to utilize
        function blocking_ts_move(self, start, pos, nvargs)
            arguments
                self Robot;
                start double;
                pos double;
                nvargs.time double = 3;
                nvargs.mode string = "cubic"
            end
            
            traj = TrajGenerator();
            traj_coeffs = [];

            for i=1:4 % Generate each axis in task space trajectories
                traj_coeffs(i, :) = traj.quinitic_traj(start(i), pos(i), 0,0,0,0,0,nvargs.time);
            end

            tic;
            while toc < nvargs.time
                pos_at_t = traj.eval_traj(traj_coeffs, toc);

                joints = self.ik_3001(pos_at_t);
                % Drive the joints 
                self.interpolate_jp(joints, 0.05);

                %self.jacobian_ik([pos_at_t(1), pos_at_t(2), pos_at_t(3), pos_at_t(4)], 0.0000001);

                pause(0.01);
            end
            pause(0.5);
        end
        
        % Open the gripper wrapper
        function open_gripper(self)
            self.writeGripper(true);
        end

        % Close the gripper wrapper
        function close_gripper(self)
            self.writeGripper(false);
        end

        function pos = getTaskSpacePos(self)
            joint_states = self.measure_js(true, false); % Get joint positions
            joint_positions = joint_states(1,:); % take only the positions
            fk_position = self.fk_3001(joint_positions(1), joint_positions(2), joint_positions(3), joint_positions(4));
            pos = [fk_position(1, 4),...
                    fk_position(2, 4),...
                    fk_position(3, 4),...
                    joint_positions(4)];  
            
        end

        %PICK_UP_BALL picks up a ball and deposits it in the correct color bin
        % Inputs:arguments
        %   pos: a [1x2] matrix representing the position of the ball in the XY
        %        frame of the robot
        %   color: a string indicating what color bin the ball should be placed
        %          in
        %   z_offset (optional): the z-position at which to begin the straight
        %                        vertical trajectory 
        %   z_ball (optional): the z-posiiton at which to stop the vertical 
        %                      trajectory and grasp the ball
        function pick_up_ball(self, pos, color, nvargs)
            arguments
                self Robot;
                pos double
                color string
                nvargs.z_offset double = 60; % keeping the z_offset consistent helps prevent collisions
                nvargs.z_ball double = 12; % Found through experimentation
            end
            
            pos = [pos(1), pos(2), nvargs.z_offset, 90]; % Assuming the wrist is down to begin with
 
            % move to above the ball and open the EE
            self.open_gripper();
            self.blocking_ts_move(self.mPreviouslySavedPosition, pos, time=2); 

            % Lower down onto the ball
            iPos = pos;
            pos(3) = nvargs.z_ball;
            self.blocking_ts_move(iPos, pos, time=0.75);

            % Grab and raise
            self.close_gripper();
            pause(0.25);

            iPos = pos;
            pos(3) = 80;
            self.blocking_ts_move(iPos, pos, time=0.75); 

            % Sort according to color
            iPos = pos;
            switch(color)
                case "green"
                    pos = [180, 180, 100, 45]; % 100 is to prevent collisions with the bins
                case "yellow"
                    pos = [180, -180, 100, 45];
                case "orange"
                    pos = [100, -180, 100, 45];
                case "red"
                    pos = [100, 180, 100, 45];
                otherwise
                    disp("Color invalid, First letter is lowercase.");
            end

            % Move to the grid and drop the ball
            self.blocking_ts_move(iPos, pos, time=1); 
            self.open_gripper();
            pause(.25);

            self.mPreviouslySavedPosition = pos;
        end
    end % end methods
end % end class 
