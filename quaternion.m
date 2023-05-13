function [Quat] = Quaternion(PHI, THETA, PSI)

q0=cos(0.5*PSI)*cos(0.5*THETA)*cos(0.5*PHI)+sin(0.5*PSI)*sin(0.5*THETA)*sin(0.5*PHI);
q1=cos(0.5*PSI)*cos(0.5*THETA)*sin(0.5*PHI)-sin(0.5*PSI)*sin(0.5*THETA)*cos(0.5*PHI);
q2=cos(0.5*PSI)*sin(0.5*THETA)*cos(0.5*PHI)+sin(0.5*PSI)*cos(0.5*THETA)*sin(0.5*PHI);
q3=cos(0.5*PSI)*sin(0.5*THETA)*sin(0.5*PHI)+sin(0.5*PSI)*cos(0.5*THETA)*cos(0.5*PHI);

Quat=[ q0; q1; q2; q3];
end

%This function allows to update quaternion values from the aircraft angles
%related to the NED frame.

