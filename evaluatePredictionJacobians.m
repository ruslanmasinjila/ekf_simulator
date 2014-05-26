%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function [ G_mut,G_ut] = evaluatePredictionJacobians(b,previousPose,ut)

%   Computes the Jacobians G_mut and G_ut
%   G_mut:  Partial derivative of the current pose w.r.t previous pose
%   G_ut:   Partial derivetive of the current pose w.r.t control input

%   INPUT:  
%   Previous Pose of the Robot (previousPose)
%   Control Input (ut)

%   OUTPUT:
%   G_mut
%   G_ut

%   BEGIN
DL=ut(1);
DR=ut(2);

previousTheta=previousPose(3);

G_mut(1,:)=[ 1, 0, -((DR+DL)/2)*sin(previousTheta + (DR - DL)/(2*b))];
G_mut(2,:)=[ 0, 1,  ((DR+DL)/2)*cos(previousTheta + (DR - DL)/(2*b))];
G_mut(3,:)=[0 0 1];



DS=DL+DR;
DT=((DR-DL)/(2*b))+previousTheta;

G_ut(1,:)=[DS*sin(DT)+2*b*cos(DT) -DS*sin(DT)+2*b*cos(DT)];
G_ut(2,:)=[-DS*cos(DT)+2*b*sin(DT) DS*cos(DT)+2*b*sin(DT)];
G_ut(3,:)=[-4 4];

G_ut=(1/(4*b))*G_ut;


%   END

end


