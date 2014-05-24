%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function [Z] = estimateRelativePose(robotPose,landmarkPose,S)

%   When S=[0 0]:   Estimates the distance and angle of landmark w.r.t robot (Z_bar)
%   Otherwise:      Simulates the actual, noisy distance and angle of landmarkPose w.r.t robot (Z)

%   INPUT:
%   Position (mu=[x y theta])
%   Landmark Position (landmarkPose=[x y])
%   Sensor covariances (S=[sigma_rho sigma_phi])
%                       sigma_rho~Distance error
%                       sigma_phi~Angle error

%   OUTPUT
%   When S=[0 0]:   Estimatd relative Pose (Z_bar)
%   Otherwise:      Simulated, actual, noisy Pose (Z)

%   BEGIN

%******************************************************************
%   Estimate the relative pose w.r.t robot

sigma_rho=S(1);
sigma_phi=S(2);

dx=landmarkPose(1)-robotPose(1);
dy=landmarkPose(2)-robotPose(2);
rho=sqrt((dx)^2+(dy)^2);

phi=atan2(dy,dx);

rho=rho+sigma_rho*randn;


%   Add noise to the measured angle
%   Find difference between measured noisy angle and robot angle (theta)
phi=(phi-robotPose(3))+sigma_phi*randn;

%   Normalize the results between [-pi,pi]
phi=normalizeAngle(phi);

Z=[rho;phi];

end

