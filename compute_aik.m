function joint_angles = compute_aik(ee_pose)

%Transformation matrix of ee position wrt frame 4, needed to find T04.
    T4e = [1, 0, 0, 0.1136;
        0, 0 -1, 0;
        0, 1, 0, 0;
        0, 0, 0, 1];

%Calculates T04 by multiplying ee_pose by the inverse of T4e
    T04 = ee_pose * inv(T4e);

    %Setting Parameters
    L1 = 0.0931;
    L2 = 0.1059;
    L3 = 0.1;
    Lm = 0.035;
    Ll = 0.1;

    %Position vector of frame 4's origin wrt frame 0's origin
    x4 = T04(1,4);
    y4 = T04(2,4);
    z4 = T04(3,4);


    %Projected distance between frame 0 and frame 4 position on the xy plane 
    d = sqrt(x4^2 + y4^2);

    %Total height from frame 0 to the frame 4 position, where z4 is the z
    %position of frame 4 and L1 is the length of the first link.
    h = z4 - L1;

    %Distance between frame 0 and frame 4 position 
    r = sqrt(d^2 + h ^2);
    



    %Calculating angles used for to solve for theta 2 and theta, coming
    %from law of cosine
    cos_alpha = (L3^2-L2^2-r^2) / (-2*L2*r);
    alpha = atan2(sqrt(1-cos_alpha^2), cos_alpha);
    cos_gamma = (r^2-L2^2-L3^2) / (-2*L2*L3);
    gamma = atan2(sqrt(1-cos_gamma^2), cos_gamma);

    beta = atan2(Lm, Ll);
    phi = pi/2 - beta;
    psi = atan2(h, d);
    
    
    %Calculating the thetas using derivation
    theta1 = atan2(-T04(1,3), T04(2,3));
    theta2up = pi/2 - beta - alpha - psi;
    %theta2down = pi/2 - (gamma - alpha + beta);
    theta3up = pi - gamma - phi;
    %theta3down = -pi + (phi - psi);
    theta234 = atan2(-T04(3,1), -T04(3,2));
    theta4 = theta234 - theta3up - theta2up;


    %Outputting a variable "joint_angles" as a 1x4 vector of the thetas    
    joint_angles = [theta1; theta2up; theta3up; theta4];
end
