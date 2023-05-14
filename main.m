%==========================================================================
                         %Initial Parameters
%==========================================================================

%IMU data called
load('IMU_data.mat')

%Constants
c= 299792458;                                                                  %Ligth Speed (m/s)
Ra= 6378137.0;                                                                 %Semi-Major Axis (m)
Rb= 6356752.3142;                                                              %Semi-Minor Axis (m)
e= sqrt(1-(Rb/Ra)^2);                                                          %Eccentricity
f= (Ra-Rb)/Ra;                                                                 %Flattening
samp= 1/500;                                                                   %Sample Time(s)

%Position
phi= 39.976419*pi/180;                                                         %Longitude Angle (rad)
lbd= 116.340561*pi/180;                                                        %Latitude Angle (rad)
h= 57;                                                                         %Height (m)
ECEF=[ phi; lbd; h];                                                           %Initial vector containing coordinate from the ECEF frame
Roll = asin(mean(fby)/mean(fbz));                                               %Initial roll angle (rad) 
Pitch = asin(mean(fbx)/mean(fbz));                                              %Initial pitch angle (rad)     
Yaw= 2*pi-75*pi/180;                                                           %Initial yaw angle (rad)  
BODY=[ Roll; Pitch; Yaw];                                                      %Initial vector containing coordinate angles of the aircraft motion from the NED coordinate

%Velocity
wie= 7.292115*10^(-5);                                                         %Earth Rotation Rate (rad/s)
VNED= [ 0; 0; 0];                                                              %Initial Velocity[ Vx, Vy, Vz] respect to the NED frame (m/s)

%Initial list for plotting
Lat= [phi]
lon= [lbd];
Rolling= [Roll];
Pitching= [Pitch];
Yawing= [Yaw];
North_V= [0];
East_V= [0];

%Quaternions
Quat = quaternion(BODY(1),BODY(2),BODY(3));                                     %Initial Quaternions  (from Quaternion.m function)
Cbn = quaternion_to_DCM(Quat);                                                  %Initial DCM (from Quaternion_to_DCM.m function)
Cnb = Cbn';                                                                     %Transposition property   
%==========================================================================
                             %Algorithm running
%==========================================================================

for i= 1:length(fbx)
    
    Fb= [ fbx(i); fby(i); fbz(i)];                                             %Accelerations data from IMU (accelerometers)
    Wb= [ wbx(i); wby(i); wbz(i)];                                             %Rotation rate data from IMU (gyroscopes)                                                  

    %Gravity update
    g0= 9.780319*(1+0.0055024*sin(ECEF(1))^2-0.0000059*sin(ECEF(1))^2);
    gvt= g0/(1+ECEF(3)/Ra);

    %Radius update
    rlbd= Ra/sqrt(1-e^2*sin(ECEF(1))^2);
    rphi= Ra*(1-e^2)/((1-e^2*sin(ECEF(1))^2)^(3/2));

    %Rotation Rates update
    wnenX= VNED(2)/(rlbd+ECEF(3));
    wnenY= -VNED(1)/(rphi+ECEF(3));
    wnenZ= -VNED(2)*tan(ECEF(1))/(rlbd+ECEF(3));
    wnen= [ wnenX; wnenY; wnenZ];

    wnie= [ wie*cos(ECEF(1)); 0; -wie*sin(ECEF(1))];
    wNED= wnen+wnie;

    wbnb= Wb-Cnb*wNED;                                                            

%==========================================================================
                     %DCM Update from quaternions
%==========================================================================

    Delta_Matrix= [0 -wbnb(1) -wbnb(2) -wbnb(3); wbnb(1) 0 wbnb(3) -wbnb(2); wbnb(2) -wbnb(3) 0 wbnb(1); wbnb(3) wbnb(2) -wbnb(1) 0]*samp;
    d_theta0= samp^2*(wbnb(1)^2+wbnb(2)^2+wbnb(3)^2);

    %Quaternions update (q0, q1, q2, q3)
    q= ((1-d_theta0^2/8+d_theta0^4/384)*eye(4)+(0.5-d_theta0^2/48)*Delta_Matrix)*Quat;

    %DCM Update        
    Cbn = quaternion_to_DCM(q);                                                     
    Cnb = Cbn';                                                                 %Transposition property    
    Pitch = asin(-Cnb(3,1));                                                    % Update of the Pitch rate of the aircraft (new THETA)
    Roll = atan(Cnb(3,2)/Cnb(3,3));                                             % Update of the Roll rate of the aircraft (new PHI)
    Yaw = atan(Cnb(2,1)/Cnb(1,1));                                              % Update of the Yaw rate of the aircraft (new PSI)
    BODY = [ Roll; Pitch; Yaw];                                                 % Vector containing angles of motion of the aircraft from the NED frame

%==========================================================================
                           %Kinematics Update
%==========================================================================

    FNED= Cbn*Fb;                                                               % Acceleration data from IMU in the NED frame (Vector with 3 components)

    AN = FNED(1)+(wNED(3)-wie*sin(phi))*VNED(2)-wNED(2)*VNED(3);                % Acceleration rate in the North axis (m/s²)
    AE = FNED(2)-(wNED(3)-wie*sin(phi))*VNED(1)+(wNED(1)+wie*cos(phi))*VNED(3); % Acceleration rate in the East axis (m/s²)
    AD = FNED(3)-(wNED(1)+wie*cos(phi))*VNED(2)+wNED(2)*VNED(1)+gvt;            % Acceleration rate in the Down axis (m/s²)

    VNED = VNED + [ AN; AE; AD]*samp;                                           % Velocity update from Acceleration rate integration (Vector with 3 components) (m/s)
    POS = ECEF_to_cart(ECEF(1),ECEF(2),ECEF(3)) + VNED*samp;                    % Position update from Velocity rate integration (Vector with 3 components) (m)
    ECEF = cart_to_ECEF(POS(1),POS(2),POS(3));                                  % ECEF data update (vector with the new values of: phi, lambda and h) 
    
%==========================================================================
                            %Plot lists update
%==========================================================================

    Lat(end+1) = ECEF(1);
    lon(end+1) = ECEF(2);
    Rolling(end+1) = BODY(1);
    Pitching(end+1) = BODY(2);
    Yawing(end+1) = BODY(3);
    North_V(end+1) = VNED(1);
    East_V(end+1) = VNED(2);
    
end

lat= rad2deg(Lat);
lon= rad2deg(lon);
Rolling= rad2deg(Rolling);
Pitching= rad2deg(Pitching);
Yawing= rad2deg(Yawing);

plot( Lat, Rolling, Lat, Pitching, Lat, Yawing, Lat, North_V, Lat, East_V, lon, Rolling, lon, Pitching, lon, Yawing, lon, North_V, lon, East_V)


