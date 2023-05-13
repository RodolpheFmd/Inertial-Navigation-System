%==========================================================================
                               %Initialisation
%==========================================================================

%IMU DATA LOADING
load('IMU_data.mat')

%Constants
c= 299792458;                                                              %Ligth Speed (m/s)
wie= 7.292115*10^(-5);                                                     %Earth Rotation Rate (rad/s)
Ra= 6378137.0;                                                             %Semi-Major Axis (m)
Rb= 6356752.3142;                                                          %Semi-Minor Axis (m)
e= sqrt(1-(Rb/Ra)^2);                                                      %Eccentricity
f= (Ra-Rb)/Ra;                                                             %Flattening
samp= 1/500;                                                               %Sample time updating (s)

%Initial Position
phi= 39.976419*pi/180;                                                     %Longitude Angle (rad)
lbd= 116.340561*pi/180;                                                    %Latitude Angle (rad)
h= 57;                                                                     %Height (m)
ECEF= [ phi; lbd; h];

%Velocity
VNED= [ 0, 0, 0];                                                          %Initial Velocity[ Vx, Vy, Vz] respect to the NED frame (m/s)
wnie=[ wie*cos(phi); 0; -wie*sin(phi)];                                    %Rotation Rate of the Earth against the inertial frame expressed in the NED frame. (rad/s)                         

%Initial Gravity (m/s²)              
g0=9.780319*(1+0.0055024*sin(phi)^2-0.0000059*sin(phi)^2); 
gvt=g0/(1+h/Ra);

rlbd= Ra/sqrt(1-e^2*sin(phi)^2);                                           %Initial Meridian Radius of curvature (m) 
rphi= Ra*(1-e^2)/((1-e^2*sin(phi)^2)^(3/2));                               %Initial Transverse Radius of curvature (m) 
       
%==========================================================================
                %Algorithm running: DCM Initialisation
%==========================================================================

g_n= [ 0; 0; gvt];                                                         %Vectorial representation of the gravity expressed in the NED frame (m/s²)
f_tilde= [ mean(fbx); mean(fby); mean(fbz)];                               %Mean value of the accelerometers data from the IMU (m/s²)                              
w_tilde= [ mean(wbx); mean(wby); mean(wbz)];                               %Mean value of the gyroscopes data from the IMU (rad/s)     
cross_gw= cross(g_n,wnie);                                                 %Cross product between g_n and wnie
cross_fw= cross(f_tilde,w_tilde);                                          %Cross product between f_tilde and w_tilde

A= inv(transpose([ g_n wnie cross_gw]));                                   %Definition of the matrix A from the Coarse Alignment Method 
B= transpose([ f_tilde w_tilde cross_fw]);                                 %Definition of the matrix B from the Coarse Alignment Method   
Cnb= A*B                                                                   %#ok<*MINV> %Initial DCM founded from AB                                                

%Euler Angles
PHI= atan(Cnb(3,2)/Cnb(3,3))                                               %Determination of the initial roll angle (PHI) from the DCM                                        
THETA= atan(-Cnb(3,1)/(Cnb(3,3)*(sin(PHI)+cos(PHI))))                      %Determination of the initial pitch angle (THETA) from the DCM 
PSI= atan(Cnb(2,1)/Cnb(1,1))                                               %Determination of the initial yaw angle (PSI) from the DCM 







