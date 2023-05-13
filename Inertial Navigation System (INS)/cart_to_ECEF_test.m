coord= [4510731, 4510731, 0];            %We have to set coordinates
x= coord(1);
y= coord(2);
z= coord(3);

r= sqrt(x^2+y^2);
p= sqrt(r^2+z^2);

%==========================================================================
                    %Constants implementation WGS84
%==========================================================================

a= 6378137.0;                  %Semi-Major Axis
b= 6356752.3142;               %Semi-Minor Axis
e= sqrt(1-(b^2/a^2));          %Eccentricity
f= (a-b)/a;                    %Flattening


%==========================================================================
  %Cartesian coordinate (x, y, z) to Geodetic coordinate (phi, lambda, h)
%==========================================================================


u= atan((z/r)*(1-f+(e^2*a/p)));
tphi= (z*(1-f)+e^2*a*sin(u)^3)/((1-f)*(r-e^2*a*cos(u)^3));

phi= atan(tphi)                                       %return phi value
lambda= rad2deg(atan(y/x))                            %return lambda value
h=r*cos(phi)+z*sin(phi)-a*sqrt(1-e^2*sin(phi)^2)      %return h value



