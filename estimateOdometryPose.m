%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function [currentPose] = estimateOdometryPose(b,previousPose,ut)

%   Computers the current pose of the robot from previous pose and control input.

%   INPUT:
%   Previous Pose (previousPose)
%   Control (ut=[DL DR])
%           DL~Left wheel displacement
%           DR~Right wheel displacement
%   Distance between robot wheels (b)


%   START
previousX=previousPose(1);
previousY=previousPose(2);
previousTheta=previousPose(3);

%   ut=[DL DR]
DL=ut(1);
DR=ut(2);

% Calculate the new x coordinate
currentX=   previousX + ((DR+DL)/2)*cos(previousTheta + ((DR-DL)/(2*b)));

% Calculate the new y coordinate
currentY=  previousY +  ((DR+DL)/2)*sin(previousTheta+ ((DR-DL)/(2*b)));

% Calculate the new Theta
currentTheta=previousTheta+(DR-DL)/b;


%   Normalize results between [-pi,pi]
currentTheta=normalizeAngle(currentTheta);


% Put them together
currentPose=[currentX;currentY;currentTheta];


%   END

end