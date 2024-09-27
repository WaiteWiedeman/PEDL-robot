function y = robot_simulation(tSpan, sysParams, ctrlParams)
    % ODE solver
    if ctrlParams.fixedTimeStep ~= 0
        tSpan = tSpan(1):ctrlParams.fixedTimeStep:tSpan(2);
    end
    x0 = zeros(6, 1); % q1, q1d, q2, q2d
    [t,x] = ode45(@(t,x) robot_system(t, x, sysParams, ctrlParams), tSpan, x0);
    % sample time points
    [t,x] = select_samples(ctrlParams, t, x);
    numTime = length(t);
    y = zeros(numTime, 17); 
    for i = 1 : numTime
        [Xd, Yd, Xdd, Ydd] = referenceTrajectory(t(i), ctrlParams);
        [Th1,Th2,~,~] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xd,Xdd);
        F = force_function(t, x(i,:), Xd, Yd, Xdd, Ydd, ctrlParams);
        xdot = robot_xdot(x(i,:), F, sysParams);
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
        y(i,14) = Xd;
        y(i,15) = Yd;
        y(i,16) = Th1;
        y(i,17) = Th2;
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
