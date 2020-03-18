function Ry = roty(beta)
% Creates rotation matrix about axis Z
% input in radians
% ct=cos(theta);
% st=sin(theta);
% Ry=[ct 0 st;0 1 0 ;-st 0 ct];

Ry = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];
end


