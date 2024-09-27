function params = params_control()
    params = struct();
    params.PID0 = [20000 10 10000]; 
    params.PID1 = [20000 10 10000];
    params.PID2 = [20000 10 10000];
    params.refx = 1;
    params.refy = 1;
    params.refrad = 0.5;
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "origin"; % random, interval, origin
    params.numPoints = 200;
    params.interval = 1e-3;
end