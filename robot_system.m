function dxdt = robot_system(t, x, sysParams, ctrlParams)
    [Xd, Yd, Xdd, Ydd, Xc, Xcd] = referenceTrajectory(t, ctrlParams,sysParams);
    [Th1,Th2,Om1,Om2] = InverseKinematics(Xd,Yd,Xdd,Ydd,Xc,Xcd);
    F = force_function(t, x, Xc, Xcd, Th1, Th2, Om1, Om2, ctrlParams);
    fc = coulomb_friction(x(2), sysParams, ctrlParams.friction);
    dxdt = robot_xdot(x, F, fc, sysParams);
end