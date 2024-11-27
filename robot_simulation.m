function y = robot_simulation(tSpan, x0, sysParams, ctrlParams)
    % ODE solver
    if ctrlParams.fixedTimeStep ~= 0
        tSpan = tSpan(1):ctrlParams.fixedTimeStep:tSpan(2);
    end
    
    switch ctrlParams.solver
        case "nonstiff"
            [t,x] = ode45(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0);
        case "stiffhr"
            opts = odeset('RelTol',1e-7,'AbsTol',1e-9); 
            [t,x] = ode15s(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0, opts); 
        case "stifflr"
            opts = odeset('RelTol',1e-4,'AbsTol',1e-7); 
            [t,x] = ode15s(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0, opts);
        case "GA"
            startTime = datetime;
            stopTime = 60; % end sim in 60 seconds
            opts = odeset('RelTol',1e-7,'AbsTol',1e-9,'OutputFcn', @(t, y, flag) myOutputFcn(t, y, flag, startTime, stopTime));
            [t,x] = ode15s(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0, opts); 
    end
    [t,x] = select_samples(ctrlParams, t, x);
    numTime = length(t);
    y = zeros(numTime, 19); 
    for i = 1 : numTime
        [Xd, Yd, Xdd, Ydd, Xc, Xcd] = referenceTrajectory(t(i), ctrlParams,sysParams);
        [Th1,Th2,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xc,Xcd);
        F = force_function(t(i), x(i,:), Xc, Xcd, Th1, Th2, Om1, Om2, ctrlParams);
        fc = coulomb_friction(x(i,2), sysParams, ctrlParams.friction);
        xdot = robot_xdot(x(i,:), F, fc, sysParams);
        y(i,1) = t(i); % t
        y(i,2) = x(i, 1); % th0
        y(i,3) = x(i, 3); % th1
        y(i,4) = x(i, 5); % th2
        y(i,5) = x(i, 2); % th0dot
        y(i,6) = x(i, 4); % th1dot
        y(i,7) = x(i, 6); % th2dot
        y(i,8) = xdot(2); % th0ddot
        y(i,9) = xdot(4); % th1ddot
        y(i,10) = xdot(6); % th2ddot
        y(i,11) = F(1); % u
        y(i,12) = F(2); % t1
        y(i,13) = F(3); % t2
        y(i,14) = fc; % Coulomb Friction
        y(i,15) = Xd; % X desired
        y(i,16) = Yd; % Y desired
        y(i,17) = Th1; % Th1 desired
        y(i,18) = Th2; % Th2 desired 
        y(i,19) = Xc; % desired cart position
    end
end

function [ts, xs] = select_samples(ctrlParams, t, x)
    switch ctrlParams.method
        case "random"
            indices = randperm(length(t), ctrlParams.numPoints);
            sortIndices = sort(indices);
            ts = t(sortIndices);
            xs = x(sortIndices,:);
        case "interval"
            ts = [t(1)];
            xs = [x(1,:)];
            for i = 2:length(t)
                if t(i)-ts(end) >= ctrlParams.interval
                    ts = [ts;t(i)];
                    xs = [xs;x(i,:)];
                end
            end
        otherwise
            ts = t;
            xs = x;
    end
end

function status = myOutputFcn(t, y, flag, startTime, stopTime)
    currentTime = datetime;
    status = double(seconds(currentTime-startTime) > stopTime);
end
