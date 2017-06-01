function [q] = myeul2quat(e1, e2, e3)
%
%Function to convert Euler angles in degrees to a quaternion. The convention used is that of Bunge (1965, 1982)
%Prior to using this euler angle should be converted to this convention
%
%Inputs:
%	e1: n x 1 vector of Euler angles 1 (phi1)
%	e2: n x 1 vector of Euler angles 2 (PHI)
%	e3:	n x 1 vector of Euler angles 3 (phi2)
%
%Outputs:
%	q: quaternion corresponding to the rotation described by Euler angles
%
%  Copyright 2014 Mark Pearce
%
%  This file is part of EBSDinterp.
%
%  EBSDinterp is free software: you can redistribute it and/or modify
%  it under the terms of the CSIRO Open Source Software Licence
%  distributed with this software as "CSIRO_BSD_MIT_License_v2_0.txt".
%
%  EBSDinterp is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU General Public License for more details.
%

%Redefine as alpha, beta and gamma in radians
al = e1 / 180 * pi - (pi/2);
be = e2 / 180 * pi;
ga = e3 / 180 * pi - (3*pi / 2);

for i = 1:length(al)
    q(i,:) = [cos(be(i)/2)*cos((al(i) + ga(i))/2); -sin(be(i)/2)*sin((al(i) - ga(i))/2); sin(be(i)/2)*cos((al(i) - ga(i))/2); cos(be(i)/2)*sin((al(i) + ga(i))/2)]';
end