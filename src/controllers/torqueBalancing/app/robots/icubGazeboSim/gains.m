
ROBOT_DOF = 32;
%ROBOT_DOF = 23;
CONFIG.ON_GAZEBO = true;
PORTS.IMU = '/icubSim/inertial';


CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT  = [1 1];

CONFIG.SMOOTH_DES_COM      = 1;    % If equal to one, the desired streamed values 
                            % of the center of mass are smoothed internally 

CONFIG.SMOOTH_DES_Q        = 1;    % If equal to one, the desired streamed values 
                            % of the postural tasks are smoothed internally 

WBT_wbiList = '(torso_pitch, torso_roll, torso_yaw, neck_pitch, neck_roll, neck_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, l_wrist_prosup, l_wrist_pitch, l_wrist_yaw, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, r_wrist_prosup, r_wrist_pitch, r_wrist_yaw, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll)';
%WBT_wbiList = '(torso_pitch, torso_roll, torso_yaw, l_shoulder_pitch, l_shoulder_roll, l_shoulder_yaw, l_elbow, r_shoulder_pitch, r_shoulder_roll, r_shoulder_yaw, r_elbow, l_hip_pitch, l_hip_roll, l_hip_yaw, l_knee, l_ankle_pitch, l_ankle_roll, r_hip_pitch, r_hip_roll, r_hip_yaw, r_knee, r_ankle_pitch, r_ankle_roll)';
WBT_robotName          = 'icubSim';

dump.left_wrench_port  = '/icubSim/left_foot/analog:o';
dump.right_wrench_port = '/icubSim/right_foot/analog:o';

references.smoothingTimeMinJerkComDesQDes    = 2;

sat.torque = 34;

CONFIG.smoothingTimeTranDynamics    = 0.05;

ROBOT_DOF_FOR_SIMULINK = eye(ROBOT_DOF);
gain.qTildeMax         = 20*pi/180;
postures = 0;  

gain.SmoothingTimeImp  = 1; 
gain.SmoothingTimeGainScheduling = 0.02;

%%
%           PARAMETERS FOR TWO FEET ON GROUND
if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 2)
    gain.PCOM                 = diag([50    50  50]);
    gain.ICOM                 = diag([0    0  0]);
    gain.DCOM                 = diag([15    15  15]);

    gain.PAngularMomentum     = 0.25;
    gain.DAngularMomentum     = 2*sqrt(gain.PAngularMomentum);


    % Impedances acting in the null space of the desired contact forces 
    
    impHead            = [10   10   10
                           0    0    0]; 
       
    impTorso            = [70   70   70
                            0    0    0]; 
                        
    impLeftArm         = [50   50    50    30    30    10    10  
                            0    0     0    0      0    0     0 ];                     
                        
    impRightArm             = [50   50    50    30    30    10    10  
                            0    0     0    0      0    0     0 ];
                        
   % impArms             = [20   20    20    20 
    %    0    0     0    0];
                        
    impLeftLeg          = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 
                         
    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]; 
    
    intHead             = [0   0    0];                  
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0   0   0   0   0];
                        
    intLeftLeg          = [0   0    0    0   0  0]; 

    intRightLeg         = [0   0    0    0   0  0];   
    
                                           
end

% PARAMETERS FOR ONLY ONE FOOT ON GROUND

if (sum(CONFIG.LEFT_RIGHT_FOOT_IN_CONTACT) == 1)
    %%
    gain.PCOM                 = diag([50   100  50]);
    gain.ICOM                 = diag([  0    0   0]);
    gain.DCOM                 = diag([  0    0   0]);

    gain.PAngularMomentum     = 1 ;
    gain.DAngularMomentum     = 1 ;

    % Impedances acting in the null space of the desired contact forces 
    
     intHead             = [0   0    0];                 
    
    intTorso            = [0   0    0]; 
    intArms             = [0   0    0   0   0   0   0];
                        
    intLeftLeg          = [0   0    0    0    0  0]; 

    intRightLeg         = [0   0    0    0    0  0];  
    
    scalingImp          = 1.5;
    
    impHead            = [20   20   20
                           0    0    0]; 
    
    impTorso            = [20   20   30
                            0    0    0]*scalingImp; 
   impLeftArm         = [20   20    20    20    20    20    20  
                            0    0     0    0      0    0     0 ]*scalingImp;                     
                        
    impRightArm             = [20   20    20    20    20    20    20  
                            0    0     0    0      0    0     0 ]*scalingImp;
                        
    impLeftLeg          = [ 30   30   30   120     10  10
                             0    0    0     0      0   0]*scalingImp; 

    impRightLeg         = [ 30   30   30    60     10  10
                             0    0    0     0      0   0]*scalingImp; 
                            
%%    
end

sat.integral              = 0;

gain.integral             = [intTorso,intHead,intArms,intArms,intLeftLeg,intRightLeg];
%gain.integral             = [intTorso,intArms,intArms,intLeftLeg,intRightLeg];

gain.impedances           = [impTorso(1,:),impHead(1,:),impLeftArm(1,:),impRightArm(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
gain.increasingRatesImp   = [impTorso(2,:),impHead(2,:),impLeftArm(2,:),impRightArm(2,:),impLeftLeg(2,:),impRightLeg(2,:)];
%gain.impedances           = [impTorso(1,:),impArms(1,:),impArms(1,:),impLeftLeg(1,:),impRightLeg(1,:)];
%gain.increasingRatesImp   = [impTorso(2,:),impArms(2,:),impArms(2,:),impLeftLeg(2,:),impRightLeg(2,:)];

gain.dampings             = 0*sqrt(gain.impedances);
sat.impedences            = [80   25    1400];

if (size(gain.impedances,2) ~= ROBOT_DOF)
    error('Dimension mismatch between ROBOT_DOF and dimension of the variable impedences. Check these variables in the file gains.m');
end


%% constraints for QP for balancing on both feet - friction cone - z-moment - in terms of f (not f0!)


% Friction cone parameters
numberOfPoints               = 4; % The friction cone is approximated by using linear interpolation of the circle. 
                                  % So, numberOfPoints defines the number of points used to interpolate the circle in each cicle's quadrant 

forceFrictionCoefficient     = 1;%1/3;  
torsionalFrictionCoefficient = 2/150;

%physical size of foot
phys.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];  % yMin, yMax    
                             
gain.footSize                = [ -0.07 0.07   ;   % xMin, xMax
                                 -0.03 0.03 ];  % yMin, yMax    

fZmin                        = 10;

%% The QP solver will search a solution fo that 
% satisfies the inequality Aineq_f F(fo) < bineq_f
reg.pinvTol     = 1e-5;
reg.pinvDamp    = 0.01;
reg.pinvDampVb  = 1e-7;
reg.HessianQP   = 1e-5;
reg.impedances  = 0.1;
reg.dampings    = 0;
