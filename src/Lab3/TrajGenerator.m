classdef TrajGenerator < handle  
    methods(Static)
        function coeffs = cubic_traj(q0, qf, v0, vf, t0, tf)
        %CUBIC_TRAJ Generates coefficients for a cubic trajectory
        % Inputs:
        %        q: [1:4] matrix with initial and final: q, velocity 
        % Outputs:
        %   coeffs: a [1x4] matrix of cubic trajectory coefficients
     
        M = [1 t0 t0^2 t0^3;
            0 1 2*t0 3*t0^2;
            1 tf tf^2 tf^3;
            0 1 2*tf 3*tf^2;];
        b = [q0; v0; qf; vf];
        coeffs = M \ b;
            
        end

        % TODO: Fill in the arguments for these methods
        function coeffs = quinitic_traj(q0, qf, v0, vf, a0, af, t0, tf)
        %CUBIC_TRAJ Generates coefficients for a quintic trajectory
        % Inputs:
        %        q: [1:6] matrix with initial and final: q, velocity, acceleration 
        % Outputs:
        %   coeffs: a [1x6] matrix of cubic trajectory coefficients
        
        M = [1  t0  t0.^2     t0.^3     t0.^4     t0.^5;
             0   1   2*t0   3*t0.^2   4*t0.^3   5*t0.^4;
             0   0      2      6*t0  12*t0.^2  20*t0.^3;
             1  tf  tf.^2     tf.^3     tf.^4     tf.^5;
             0   1   2*tf   3*tf.^2   4*tf.^3   5*tf.^4;
             0   0      2      6*tf  12*tf.^2  20*tf.^3];
        b = [q0; v0; a0; qf; vf; af];
        coeffs = M \ b;
        
        end

        function state = eval_traj(coeff_mat, t) 
        %EVAL_TRAJ Evaluates multiple trajectories
        % Inputs:
        %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
        %              cubic or quintic trajectory coefficients
        %   t: a time in seconds at which to evaluate the trajectories
        % Outputs:
        %   state: a [nx1] column vector containing the results of 
        %          evaluating the input trajectories at time t
        %          state will be joint space, so 4 thetas,
        
        state = zeros(size(coeff_mat, 1), 1); % make the state vector nx1
        for i = 1:size(coeff_mat, 1)
            state(i) = polyval(flip(coeff_mat(i, :)), t);
        end

        end

        function state = eval_traj_vel(coeff_mat, t) 
        %EVAL_TRAJ Evaluates multiple trajectories
        % Inputs:
        %   coeff_mat: a [nx4] or [nx6] matrix where each row is a set of
        %              cubic or quintic trajectory coefficients
        %   t: a time in seconds at which to evaluate the trajectories
        % Outputs:
        %   state: a [nx1] column vector containing the results of 
        %          evaluating the input trajectories at time t
        %          state will be joint space, so 4 thetas,
        
        state = zeros(size(coeff_mat, 1), 1); % make the state vector nx1
        for i = 1:size(coeff_mat, 1)
            state(i) = polyder(flip(coeff_mat(i, :)), t);
        end

        end

    end
end