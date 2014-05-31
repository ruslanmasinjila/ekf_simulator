%*************************************************************
%   AUTHOR:     Ruslan Masinjila
%   Contact:    ruslanmasinjila@gmail.com
%*************************************************************
function ekfSimulation()

%   Main function for simulating the localization of the Robot using
%   Extended Kalman Filter

%**************************************************************************
%   SIMULATION PROPERTIES
%   ##CHANGE THESE VALUES FOR DIFFERENT RESULTS##

%   Simulation Steps
simSteps=200;

%**************************************************************************
%   ROBOT PROPERTIES
%   Distance between robot wheels
b=0.5;

%**************************************************************************
%   SENSOR PROPERTIES

%   Error in distance measurement
sigma_rho=0.001;

%   Error in angle measurement
sigma_phi=0.1;

%   S0 is used when estimating relative position and orientation of the
%   landmark.
%   S is to simulate actual, noisy measurement of the orientation and
%   position of the landmark w.r.t the robot
S0=[0 0];
S=[sigma_rho sigma_phi];
Qt=getSensorCovariance(S);
%**************************************************************************
%   CONTROL PROPERTIES

%   Odometry noise (0.1 means 10% error)
KL=0.1;
KR=0.1;
K=[KL KR];

%   Control inputs. DL and DR should be close to b, distance between
%   wheels. That is, the robot should move only as far as the distance
%   between its wheels before taking landmark measurements.
DL=b;
DR=b;



%**************************************************************************
%   POSE
% Initialize location matrices
% first column=x
% second column=y
% third column=theta.

% The expected pose is the pose of the robot determined using only internal
% encoders assuming there is no noise. This is the "Perfect Pose"
mu_expected=zeros(simSteps,3);
mu_expected(1,:)=[0 0 pi];


% The actual pose is the real pose of the robot.
mu_actual=mu_expected;

% mu is the filtered pose
mu=mu_expected;

% Keeps track of the total distance covered in X and Y axix
totalDL=0;
totalDR=0;
distanceCovered=struct;


%   Initializes covariance matrix. Diagonal elements sigma(1,1), sigma(2,2)
%   and sigma(3,3) are the covariances for x,y and theta respectivelly.
sigma=cell(simSteps,1);
sigma{1}=[0 0 0;0 0 0;0 0 0];

%**************************************************************************

for t=2:simSteps
    
    %   Simulate random motion
    DL=DL+0.001*randn;
    DR=DR+0.001*randn;
    %***********************************
    %   NOISY MOTION
    %   Add Gaussian Noise to Control input ut
    noisyDL=DL+(DL*KL)*randn;
    noisyDR=DR+(DR*KR)*randn;
    noisyUt=[noisyDL noisyDR];
    
    totalDL=totalDL+noisyDL;
    totalDR=totalDR+noisyDR;
    
    %   Actaul Position taking noise into account.
    mu_actual(t,:)=estimateOdometryPose(b,mu_actual(t-1,:),noisyUt);
    
    %   Place a landmark within 10 meters of the new positon of the robot
    landmarkPose=[mu_actual(t,1)+10, mu_actual(t,2)+10];
    
    %***********************************
    %   NOISLESS MOTION
    %   Estimate Noiseless Pose from odometry
    
    ut=[DL DR];    
    
    %   Compute the current noiseless pose from previous noiseless Pose
    mu_expected(t,:)=estimateOdometryPose(b,mu_expected(t-1,:),ut);
    
    %   Estimate the predicted (priori) state of the robot  from previous filtered pose and control input ut
    mu_bar=estimateOdometryPose(b,mu(t-1,:),ut);
    
    %   Compute the partial derivatives of the predicted state w.r.t control input(G_ut) and w.r.t previous
    %   corrected state (G_mut).
    [G_mut, G_ut]=evaluatePredictionJacobians(b,mu(t-1,:),ut);
    
    
    %   Determine the covariance Matrix of the wheel encoders
    Rt=getOdometryCovariance(ut,K);
    
    %   Estimate the predicted (priori) Covariance (Sigma Bar)
    sigma_bar=G_mut*sigma{t-1}*(G_mut)' + G_ut*Rt*(G_ut)';

    
    %   Estimate noisy, actual measurement Z
    Z=estimateRelativePose(mu_actual(t,:),landmarkPose,S);
    
    %   Estimate Z_bar
    Z_bar=estimateRelativePose(mu_bar,landmarkPose,S0);
    
    %   Estimate Z_diff
    Z_diff=evaluateRelativePoseDifference(Z,Z_bar);
    
    %   Estimate the partial derivative of the observation/measurement w.r.t to
    %   predicted state (priori)
    Hr=evaluateMeasurementJacobians(mu_bar, landmarkPose);
    
    %   Calculate innovation (residula) covariance
    S=  ((Hr)*(sigma_bar)*(Hr')+Qt);
    
    %   Calculate Kalman Gain (Kgain)
    Kgain=  ((sigma_bar)*(Hr'))/S;
    
    %   Update mean
    mu(t,:)=mu_bar+(Kgain*Z_diff);

    %   Normalize theta to [-pi,pi]
    mu(t,3)=normalizeAngle(mu(t,3));
    
    %   Update Covariance
    sigma{t}=(eye(3,3)-Kgain*Hr)*sigma_bar;
    
end

makePlots(mu_actual, mu_expected, mu);

%   ESTIMATE ERRORS
ekfPerformance( mu_expected, mu_actual, mu,sigma{simSteps} );

%   Display Total Distance Covered by Left and Right Wheels
disp('*****************************************************');
disp('Distance Travelled');
distanceCovered.LeftWheel=totalDL;
distanceCovered.RightWheel=totalDR;
disp(distanceCovered);


end


%   For ploting results
function makePlots(mu_actual, mu_expected, mu)

subplot(1,2,1);


plot(mu(:,1),mu(:,2),'rx-');
hold on;

plot(mu_expected(:,1),mu_expected(:,2),'bx-');
hold on;

plot(mu_actual(:,1),mu_actual(:,2),'gx-');

xlabel('X(m)');
ylabel('Y(m)');
title('XY Position of robot');

legend('EKF Position','Odometry Position','Actual Position');

subplot(1,2,2);
x=1:length(mu_actual);

plot(x,mu(:,3).*(180/pi),'rx');
hold on;



plot(x,mu_expected(:,3).*(180/pi),'bx');
hold on;


plot(x,mu_actual(:,3).*(180/pi),'gx');

xlabel('Iterations');
ylabel('Theta (rad)');
title('Robot Orientation');

legend('EKF Orientation','Odometry Orientation','Actual Orientation');


end

