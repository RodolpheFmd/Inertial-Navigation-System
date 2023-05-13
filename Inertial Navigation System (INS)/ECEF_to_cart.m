function [Cart] = ECEF_to_Cart(phi,lambda,h)

Ra= 6378137.0;                                                             %Semi-Major Axis (m)
Rb= 6356752.3142;                                                          %Semi-Minor Axis (m)
e= sqrt(1-(Rb/Ra)^2);                                                      %Eccentricity
f= (Ra-Rb)/Ra;                                                             %Flattening

N= Ra/sqrt(1-(e*sin(phi))^2);
x= (N+h)*cos(phi)*cos(lambda);
y=(N+h)*cos(phi)*sin(lambda);
z=((1-e^2)*N+h)*sin(phi);

Cart=[ x; y; z];
end

