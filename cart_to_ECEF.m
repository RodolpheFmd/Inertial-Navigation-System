function [ECEF] = Cart_to_ECEF( x, y, z)

Ra= 6378137.0;                                                             %Semi-Major Axis (m)
Rb= 6356752.3142;                                                          %Semi-Minor Axis (m)
e= sqrt(1-(Rb/Ra)^2);                                                      %Eccentricity
f= (Ra-Rb)/Ra;                                                             %Flattening

r= sqrt(x^2+y^2);
p= sqrt(r^2+z^2);

u= atan((z/r)*(1-f+(e^2*Ra/p)));
tphi= (z*(1-f)+e^2*Ra*sin(u)^3)/((1-f)*(r-e^2*Ra*cos(u)^3));

phi= atan(tphi);                                                           %return phi value
lambda= atan(y/x);                                                         %return lambda value
h= r*cos(phi)+z*sin(phi)-Ra*sqrt(1-e^2*sin(phi)^2);                        %return h value
ECEF=[ phi; lambda; h];

end
