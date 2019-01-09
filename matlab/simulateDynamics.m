function dx = simulateDynamics(t, x, F, p1, p2, p3, p4, inv_I, mass)

    % Statespace:
    % x = [position, euler angles, velocity, angular rate]

    A = zeros(12,12);
    A(1:3,7:9) = eye(3);
    A(4:6,10:12) = rot(x(4:6));
    
    B = zeros(12,12);
    B(7:9,:) = [eye(3)*1/mass eye(3)*1/mass eye(3)*1/mass eye(3)*1/mass];
    B(10:12,:) = [inv_I*skew(p1-x(1:3,1)) inv_I*skew(p2-x(1:3,1))... 
                  inv_I*skew(p3-x(1:3,1)) inv_I*skew(p4-x(1:3,1))];
              
    g = zeros(12,1);
    g(9,1) = 9.81;
      
    dx = A*x + B*F - g;
      
    function S = skew(x)
        S = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
    end
  
    function R = rot(x)
        R = [cos(x(3))/cos(x(2))  sin(x(3))/cos(x(2))  0;
                 -sin(x(3))        cos(x(3))     0;
             cos(x(3))*tan(x(2))  sin(x(3))*tan(x(2))  1];
    end
end