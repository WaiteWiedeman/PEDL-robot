function params = params_control()
    % reference parameters
    sysParams = params_system();
    l1 = sysParams.L1;
    l2 = sysParams.L2;
    
    params = struct();
    % PID gains for each input (Kp, Ki, Kd)
    params.PID0 = load('highfrictioncontroller.mat').BestChrom(1:3); %[150 20 100]; %[4116.44864113396	833.261442554813	2602.63619793509]; 
    params.PID1 = load('highfrictioncontroller.mat').BestChrom(4:6); %[10 10 10]; %[4326.11611971488/4	2268.34155272639	249.811341151034];
    params.PID2 = load('highfrictioncontroller.mat').BestChrom(7:9); %[10 10 10]; %[4915.80153387096/3	2466.27431468036	881.468595598916];
    params.refx = 1; % center of reference trajectory in x
    params.refy = 1; % center of reference trajectory in y
    params.a = 4; % width of reference point range
    params.b = l1 + l2; % height of reference point range
    params.refrad = 0.5; % radius of reference trajectory 
    params.friction = "andersson"; % none, smooth, andersson, specker
    params.fixedTimeStep = 0; % 0 for varying time step, else for fixed stime step in simulation e.g., 1e-2
    % To many data points will be generated if using default ode options
    % To select small set of data for training with different methods.
    params.method = "origin"; % random, interval, origin
    params.numPoints = 100;
    params.interval = 1e-3;
    params.solver = "stifflr"; % "stifflr" (low-res) or "stiffhr" (high-res) or "nonstiff"
end