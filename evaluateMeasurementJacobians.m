%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function [ Hr ] = evaluateMeasurementJacobians( currentPose, landmarkPose)
%   Computes the Jacobians Hr and Hl
%   Hr:     Partial derivative of the Estimated Relative Pose w.r.t current pose


%   INPUT:  
%   Current Pose of the Robot (currentPose)
%   Landmark Pose (landmarjPose)

%   OUTPUT:
%   Hr


%   BEGIN

    lx=landmarkPose(1);
    ly=landmarkPose(2);
    
    rx=currentPose(1);
    ry=currentPose(2);
    
    q=sqrt((lx-rx)^2+(ly-ry)^2);
    Hr(1,:)=(1/q)*[-(lx-rx) -(ly-ry) 0];
    Hr(2,:)=(1/q)*[(ly-ry)/q -(lx-rx)/q -q];
       
%   END

end

