function F = force_function(t, x, Xc, Xcd, Th1, Th2, Om1, Om2, ctrlParams)
    persistent ti e0 e1 e2
    if t == 0
        ti = [];
        e0 = [];
        e1 = [];
        e2 = [];
    end
    ti(end+1) = t;

    F = zeros(3,1);

    th0 = x(1);
    th0d = x(2);
    th1 = x(3);
    th1d = x(4);
    th2 = x(5);
    th2d = x(6);

    PID0 = ctrlParams.PID0;
    PID1 = ctrlParams.PID1;
    PID2 = ctrlParams.PID2;

    e0(end+1) = Xc - th0;
    e0d = Xcd - th0d;
    F(1) = PID0(1)*e0(end) + PID0(2)*trapz(ti,e0,2) + PID0(3)*e0d;
    if F(1) > 200
        F(1) = 200;
    elseif F(1) < -200
        F(1) = -200;
    end

    e1(end+1) = Th1 - th1;
    e1d = Om1 - th1d;
    F(2) = PID1(1)*e1(end) + PID1(2)*trapz(ti,e1,2) + PID1(3)*e1d;
    if F(2) > 200
        F(2) = 200;
    elseif F(2) < -200
        F(2) = -200;
    end

    e2(end+1) = Th2 - th2;
    e2d = Om2 - th2d;
    F(3) = PID2(1)*e2(end) + PID2(2)*trapz(ti,e2,2) + PID2(3)*e2d;
    if F(3) > 200
        F(3) = 200;
    elseif F(3) < -200
        F(3) = -200;
    end
end
