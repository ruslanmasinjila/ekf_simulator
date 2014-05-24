%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function ekfPerformance( mu_expected, mu_actual, mu, sigma )

%   Calculates the root-mean-squared error and Standard Deviation of the filtered pose (mu).
%   Calculates the root-mean-squared error of the odometry pose (mu_expected).
%   Prints the results.

%   INPUT:
%   Odometry Pose (mu_expected)
%   Actual Pose (mu_actual)
%   Filtered Pose (mu)
%   Covariance Matrix (sigma)

%   OUTPUT:
%   Print rms error for filtered pose
%   Print standard deviation for filtered pose.
%   Print rms error for odometry pose
    


%   Initialize Structures
ekfError=struct;
odometryError=struct;
ekfSTD=struct;

%   Get the standard Deviations from the diagonal elements of the
%   Covariance Matrix. sigma(1,1),sigma(2,2) and sigma(3,3) correspond to
%   covariances for for x, y and theta respectivelly
standardDeviations=sqrt(diag(sigma));


%   rms error in the x axis of the filtered pose
ekfErrorX=sqrt(mean((mu_actual(:,1)-mu(:,1)).^2));
ekfError.X=ekfErrorX;

%   Standard deviation in x axis
ekfSTD.DX=standardDeviations(1);

%   rms error in the y axis of the filtered pose
ekfErrorY=sqrt(mean((mu_actual(:,2)-mu(:,2)).^2));
ekfError.Y=ekfErrorY;

%   Standard deviation in the y axix
ekfSTD.DY=standardDeviations(2);

%   Normalize the difference between [-pi,pi]
ekfThetaDiff=mu_actual(:,3)-mu(:,3);
for i=1:length(ekfThetaDiff)
ekfThetaDiff(i)=normalizeAngle(ekfThetaDiff(i));
end

%   rms error in Orientation of the filtered pose
ekfErrorTheta=sqrt(mean((ekfThetaDiff*(180/pi)).^2));
ekfError.Theta=ekfErrorTheta;
ekfSTD.DTheta=standardDeviations(3)*(180/pi);

%*****************************************************

%   rms error in the x axis of the odometry pose
odometryErrorX=sqrt(mean((mu_actual(:,1)-mu_expected(:,1)).^2));
odometryError.X=odometryErrorX;

%   rms error in the y axis of the odometry pose
odometryErrorY=sqrt(mean((mu_actual(:,2)-mu_expected(:,2)).^2));
odometryError.Y=odometryErrorY;


%   Normalize the difference between [-pi,pi];
odometryThetaDiff=mu_actual(:,3)-mu_expected(:,3);
for i=1:length(odometryThetaDiff)
odometryThetaDiff(i)=normalizeAngle(odometryThetaDiff(i));
end

%   rms error in the orientation of the odometry pose
odometryErrorTheta=sqrt(mean((odometryThetaDiff*(180/pi)).^2));
odometryError.Theta=odometryErrorTheta;

%*****************************************************

%   DISPLAY RESULTS
disp('*****************************************************');
disp('EKF Errors');
disp(ekfError);

disp('Standard Deviation');
disp(ekfSTD);
disp('*****************************************************');
disp('Odometry Errors');
disp(odometryError);



end

