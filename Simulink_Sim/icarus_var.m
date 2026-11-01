%Mass
m_dry = 2.5;  %kg rocket without propellant
m_prop = 0.35;  %kg propellant mass


%Aero
d = 0.098; %m body diameter
A = pi * (d/2) ^ 2;   %m^2 reference area
Cd = 0.5;  %from OpenRocket


%Environment
g = 9.81;  %m/s^2 gravity


%Atmosphere Constants
T0 = 288.15; %Kelvin
L = 0.0065;  %Kelvin / meter
P0 = 101325; %Pascals
R = 287.05; %J/kg * K
p0 = P0/(R * T0); %


%Motor thrust curve (time in s, thrust in N)
thrust_t = [0 0.05 0.2 1.0 1.8 2.0];
thrust_F = [0 300 250 230 180 0];
I_total = trapz(thrust_t, thrust_F); % N*s computed from curve


%Ignition Delay
Ignition_Delay = 2;


%Error And Oversampling Rate
%Consult the datasheets
%BMP581 Barometer/Altimeter (16x pressure oversampling)
P_RMS_Noise = 0.25;%Pa
P_ODR_Hz = 80; %Hz
P_ODR_Sample_Time = 1/P_ODR_Hz;
P_Variance = P_RMS_Noise^2;% Pa^2
P_Abs_Accuracy = 75; %+- 75 Pa


%ADXL375
ADXL_ODR_Hz = 800;
ADXL_Bandwidth = ADXL_ODR_Hz/2;
ADXL_RMS_Noise = g*(5*sqrt(ADXL_Bandwidth)/1000 ); %100 mg
ADXL_Variance = ADXL_RMS_Noise^2;
ADXL_ODR_Sample_Time = 1/ADXL_ODR_Hz;
ADXL_Sample_Time = 1/ADXL_ODR_Hz;
ADXL_Bias = 0.400*g; %400 mG
ADXL_LSB_Size = 0.049*g; %49 mg
ADXL_Range = 200*g; %200g


%BNO086
BNO_Accel_Accuracy = 0.3; %Could not find RMS error value so will use this.
BNO_Accel_Variance = BNO_Accel_Accuracy^2;
BNO_Accel_Report_Rate = 500;
BNO_Accel_Sample_Time = 1/BNO_Accel_Report_Rate;
BNO_Accel_LSB = 1/256;
BNO_Accel_Range = 8*g;
BNO_Accel_Bias = 0; %m/s^2


%Stateflow Constants
Apogee_Drop = 1; %meters
Liftoff_Hold = 0.25; %seconds
Value_Hold = 0.05; %seconds
Timeout_Hold = 1; %seconds
Backup_CoastTime = 11; %seconds
Boost_Trigger = 30; %m/s^2
Coast_Trigger = 20; %m/s^2
Main_Trigger = 300; %meters
Arm_Alt = 100; %meters
Landed_Trigger = 2; %seconds
Tolerance = 3; % meters
CdA_drogue = (2 * m_dry * g) / (p0 * 25^2); %m^2
CdA_main = (2 * m_dry * g) / (p0 * 6^2); %m^2
Bump_Amp = 0; %m/s^2 (0 = no bump)
Bump_Width = 0.01; %s
Bump_Time = 1; %s


%Kalman Filter
deltaT = 0.01; %discrete time frames of sim (chart sample time)
A_KF = [1 deltaT; 0 1]; %2x2 matrix
B_KF = [(0.5 * deltaT^2); deltaT]; %2x1 matrix
C_KF = [1 0]; %1x2 matrix (measured from barometer)
R_KF = P_Variance * (1/(p0*g))^2;   %Measurement noise 
Q_KF = B_KF * BNO_Accel_Variance * transpose(B_KF); %Process noise
