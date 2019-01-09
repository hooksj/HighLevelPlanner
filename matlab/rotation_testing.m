clear all; clc;

R1 = rot([0.1, 0.1, 0.1]);
R2 = rot([0.11,0.09, 0.1]);

R = R1*R2';

logm(R)

theta = acos(0.5*(trace(R)-1));
test = (1/(2*sin(theta)))*(R-R')*theta

% R = R1*R2';
% 
% theta1 = acos((trace(R1)-1)/2);
% theta2 = acos((trace(R2)-1)/2);
% 
% w1 = (1/(2*sin(theta1))).*[(R1(3,2)-R1(2,3));(R1(1,3)-R1(3,1));(R1(2,1)-R1(1,2))];
% w2 = (1/(2*sin(theta2))).*[(R2(3,2)-R2(2,3));(R2(1,3)-R2(3,1));(R2(2,1)-R2(1,2))];
% 
% w1-w2
% 
% expm(skew(w1))
% 
% R1 - expm(skew(w1))

function R = rot(x)
   R = [cos(x(3))*cos(x(2)) cos(x(3))*sin(x(2))*sin(x(1))-sin(x(3))*cos(x(1)) cos(x(3))*sin(x(2))*cos(x(1))+sin(x(3))*sin(x(1));
        sin(x(3))*cos(x(2)) sin(x(3))*sin(x(2))*sin(x(1))+cos(x(3))*cos(x(1)) sin(x(3))*sin(x(2))*cos(x(1))-cos(x(3))*sin(x(1));
        -sin(x(2))                        cos(x(2))*sin(x(1))                               cos(x(2))*cos(x(1))                ];
end

function S = skew(x)
  S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end