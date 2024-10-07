function params = params_control()
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    params.PID0 = [12000 10 10000]; 
    params.PID1 = [12000 10 10000];
    params.PID2 = [12000 10 10000];
    params.refx = 1; % center of reference trajectory in x
    params.refy = 1; % center of reference trajectory in y
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = "andersson"; % none, smooth, andersson, specker
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "random"; % random, interval, origin
    params.numPoints = 1000;
    params.interval = 1e-3;
end