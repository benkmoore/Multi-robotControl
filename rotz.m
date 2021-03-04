function Rz = rotz(beta)
% Creates rotation matrix about axis Z
% input in radians
% ct=cos(theta);
% st=sin(theta);
% Rz=[ct -st 0;st ct 0 ;0 0 1 ];

Rz = [cosd(beta) -sind(beta) 0; sind(beta) cosd(beta) 0; 0 0 1];
end

