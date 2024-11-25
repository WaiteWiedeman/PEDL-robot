function params = params_control()
    % reference parameters
    sysParams = params_system();
    l1 = sysParams.L1;
    l2 = sysParams.L2;
    
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    params.PID0 = [4116.44864113396	833.261442554813	2602.63619793509]; 
    params.PID1 = [4326.11611971488	2268.34155272639	249.811341151034];
    params.PID2 = [4915.80153387096	2466.27431468036	881.468595598916];
    params.refx = 1; % center of reference trajectory in x
    params.refy = 1; % center of reference trajectory in y
    params.a = 5; % width of reference point range
    params.b = l1 + l2; % height of reference point range
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = "andersson"; % none, smooth, andersson, specker
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "random"; % random, interval, origin
    params.numPoints = 500;
    params.interval = 1e-3;
    params.solver = "stiff"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
end