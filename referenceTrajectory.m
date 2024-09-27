function [Xd, Yd, Xdd, Ydd] = referenceTrajectory(t,ctrlParams)
    centerx = ctrlParams.refx;
    centery = ctrlParams.refy;
    rad = ctrlParams.refrad;
    Xd = centerx + rad*sin((2*pi/5)*t + pi/2);
    Yd = centery + rad*cos((2*pi/5)*t + pi/2);
    Xdd = -(2*pi/5)*rad*sin((2*pi/5)*t);
    Ydd = -(2*pi/5)*rad*cos((2*pi/5)*t);
end
