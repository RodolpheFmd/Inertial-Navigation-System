function [Cnb] = Quaternion_to_DCM(q)
C11= q(1)^2 + q(2)^2 - q(3)^2 - q(4)^2;
C12= 2* (q(2)*q(3)+q(1)*q(4));
C13= 2*(q(2)*q(4)-q(1)*q(3));
C21= 2*(q(2)*q(3)-q(1)*q(4));
C22= q(1)^2-q(2)^2+q(3)^2-q(4)^2;
C23= 2*(q(3)*q(4)+q(1)*q(2));
C31= 2*(q(2)*q(4)+q(1)*q(3));
C32= 2*(q(3)*q(4)-q(1)*q(2));
C33= q(1)^2-q(2)^2-q(3)^2+q(4)^2;

Cnb=[ C11 C12 C13; C21 C22 C23; C31 C32 C33];
end

%Function allows to update DCM from Quaternion values.

