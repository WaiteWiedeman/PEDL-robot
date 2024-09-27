function F = force_function(t, x, Xd, Yd, Xdd, Ydd, ctrlParams)
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

    e0 = Xd - th0;
    e0d = Xdd - th0d;
    F(1) = PID0(1)*e0 + PID0(2)*trapz(e0) + PID0(3)*e0d;

    [Th1,Th2,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xd,Xdd);

    e1 = Th1 - th1;
    e1d = Om1 - th1d;
    F(2) = PID1(1)*e1 + PID1(2)*trapz(e1) + PID1(3)*e1d;

    e2 = Th2 - th2;
    e2d = Om2 - th2d;
    F(3) = PID2(1)*e2 + PID2(2)*trapz(e2) + PID2(3)*e2d;
end

