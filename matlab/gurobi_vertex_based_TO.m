function gurobi_vertex_based_TO()

% ========================================================================
% This is a high level planner that will pass footstep locations and a
% desired CM trajectory to a low level MPC. 
% 
% - End conditions and foot a footstep plan is created based off of forward
%   integration of the user input commands.
%
% - These conditions are passed to a QP solved by gurobi to come up with a
%   feasible CM trajectory for the given conditions.
% 
% USER INPUTS
% w - Positive Y
% s - Negative Y
% d - Positive X
% a - Negative X
% j - Positive yaw
% d - Negative yaw
% q - Quit
%
% ========================================================================

    % Setup the structures for handling user inputs
    KeyStatus = false(1,7);
    KeyNames = {'w', 's','a', 'd', 'j', 'k', 'q'};
    KEY.FORWARD = 1;
    KEY.BACKWARD = 2;
    KEY.LEFT = 3;
    KEY.RIGHT = 4;
    KEY.ROT_LEFT = 5;
    KEY.ROT_RIGHT = 6;
    KEY.QUIT = 7;

    % Function to create yaw rotation matrix
    yaw_rot = @(x) [cos(x), -sin(x); sin(x) cos(x);];

    % Command inputs
    yaw_dot = 0.0; % inertial frame
    x_dot = 0.0;   % Robot frame
    y_dot = 0.0;   % Robot frame
    
    % Limits on velocity and acceleration parameters
    max_x_dot = 0.5;
    max_y_dot = 1.5;
    max_yaw_dot = 1.5;
    x_accel = 0.1;
    y_accel = 0.1;
    yaw_accel = 0.2;
    x_decel = 0.5;
    y_decel = 0.5;
    yaw_decel = 0.5;

    % Look ahead timing parameters
    step_time = 0.25;
    T_final = 2*step_time;
    dt = 0.001;
    np = T_final/dt;
    t = linspace(0.0, T_final, np);

    % State vector in the inertial frame
    x = [0.0; 0.0; 0.5; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];

    % Vectors for storing X and Y positions
    X = zeros(10*np,1);
    Y = zeros(10*np,1);
    
    % Body dimensions
    body = [0.1, -0.1, -0.1, 0.1;
            0.2, 0.2, -0.2, -0.2];
        
    % Foot parameters
    t1 = [0.0, step_time];
    t2 = [step_time, 2.0*step_time];
    t3 = [0.0, step_time];
    t4 = [step_time, 2.0*step_time];
    
    c = [1, 1, 1, 1];
    
    p1_nom = [0.15; 0.25];
    p2_nom = [-0.15; 0.25];
    p3_nom = [-0.15; -0.25];
    p4_nom = [0.15; -0.25];
    
    p1 = p1_nom;
    p2 = p2_nom;
    p3 = p3_nom;
    p4 = p4_nom;

    % Setup figure for graphical display
    % - Callback functions are used to detect user inputs
    figure('Name', 'ALPHRED V3 Simulation', 'KeyPressFcn', @MyKeyDown, 'KeyReleaseFcn', @MyKeyUp);
    axis([-1.5 1.5 -1.5 1.5]);
    H = patch(body(1,:),body(2,:), 'red');
    foot1 = animatedline('Marker','x');
    foot2 = animatedline('Marker','x');
    foot3 = animatedline('Marker','x');
    foot4 = animatedline('Marker','x');
    addpoints(foot1, p1(1,1), p1(2,1));
    addpoints(foot2, p2(1,1), p2(2,1));
    addpoints(foot3, p3(1,1), p3(2,1));
    addpoints(foot4, p4(1,1), p4(2,1));
    
    x_animate = x;
    
    X = zeros(length(x),np);

    %% Main loop for simulation
    while 1
        % Booleans determining if the robot should start decelerating
        y_decelerate = 1;
        x_decelerate = 1;
        yaw_decelerate = 1;

        % ============================
        % Check User Inputs
        %=============================
        if KeyStatus(KEY.FORWARD)
            y_decelerate = 0;
            y_dot = y_dot + y_accel;
            if y_dot > max_y_dot
                y_dot = max_y_dot;
            end
        end
        
        if KeyStatus(KEY.BACKWARD)
            y_decelerate = 0;
            y_dot = y_dot - y_accel;
            if y_dot < -max_y_dot
                y_dot = -max_y_dot;
            end
        end
        
        if y_decelerate
            if sign(y_dot)*y_dot < y_decel
               y_dot = 0.0; 
            end
            if sign(y_dot)*y_dot > 0.0
               y_dot = y_dot - sign(y_dot)*y_decel;
            end
        end
        
        if KeyStatus(KEY.RIGHT)
            x_decelerate = 0;
            x_dot = x_dot + x_accel;
            if x_dot > max_x_dot
                x_dot = max_x_dot;
            end
        end
        
        if KeyStatus(KEY.LEFT)
            x_decelerate = 0;
            x_dot = x_dot - x_accel;
            if x_dot < -max_x_dot
                x_dot = -max_x_dot;
            end
        end
        
        if x_decelerate
            if sign(x_dot)*x_dot < x_decel
               x_dot = 0.0; 
            end
            if sign(x_dot)*x_dot > 0.0
               x_dot = x_dot - sign(x_dot)*x_decel;
            end
        end
        
        if KeyStatus(KEY.ROT_LEFT)
            yaw_decelerate = 0;
            yaw_dot = yaw_dot + yaw_accel;
            if yaw_dot > max_yaw_dot
                yaw_dot = max_yaw_dot;
            end
        end
        
        if KeyStatus(KEY.ROT_RIGHT)
            yaw_decelerate = 0;
            yaw_dot = yaw_dot - yaw_accel;
            if yaw_dot < -max_yaw_dot
                yaw_dot = -max_yaw_dot;
            end
        end
        
        if yaw_decelerate
            if sign(yaw_dot)*yaw_dot < yaw_decel
               yaw_dot = 0.0; 
            end
            if sign(yaw_dot)*yaw_dot > 0.0
               yaw_dot = yaw_dot - sign(yaw_dot)*yaw_decel;
            end
        end
        
        if KeyStatus(KEY.QUIT)
            close all;
           break 
        end
        
        % integrate forward to find final state
        for i = 1:np
            % Linear
            x(4:5,1) = yaw_rot(x(9,1))*[x_dot;y_dot];
            x(1:2,1) = x(1:2,1) + x(4:5,1)*dt;

            % Rotational
            x(12,1) = yaw_dot;
            x(9,1) = x(9,1) + x(12,1)*dt;
            
            if i == 25
                x_animate = x;
            end
            
            X(:,i) = x;
        end
        
        
        % Animation
        points = zeros(2,4);
        for i = 1:4
            points(:,i) = x_animate(1:2,1)+yaw_rot(x_animate(9,1))*(body(:,i));
        end
        set(H, 'XData', points(1,:));
        set(H, 'YData', points(2,:));
        axis([-1.5+x_animate(1,1), 1.5+x_animate(1,1),...
            -1.5+x_animate(2,1), 1.5+x_animate(2,1)]);
        drawnow;
        
        pause(0.025);
        
        % Set the current state back to the animation state
        x = x_animate;
        
        t1 = t1 - [0.025, 0.025];
        if t1(1) < 0.0
           if c(1) == 0
              c(1) = 1; 
              p1 = x_animate(1:2,1)+yaw_rot(x_animate(9,1))*(p1_nom) + ...
                  yaw_rot(x_animate(9,1))*[x_dot;y_dot]*step_time*0.5;
              addpoints(foot1,p1(1,1),p1(2,1));
           else
              c(1) = 0;
              clearpoints(foot1);
           end
           t1(1) = t1(2);
           t1(2) = 2.0*step_time - 0.025;
        end
        
        t2 = t2 - [0.025, 0.025];
        if t2(1) < 0.0
           if c(2) == 0
              c(2) = 1; 
              p2 = x_animate(1:2,1)+yaw_rot(x_animate(9,1))*(p2_nom) + ...
                  yaw_rot(x_animate(9,1))*[x_dot;y_dot]*step_time*0.5;
              addpoints(foot2,p2(1,1),p2(2,1));
           else
              c(2) = 0;
              clearpoints(foot2);
           end
           t2(1) = t2(2);
           t2(2) = 2.0*step_time - 0.025;
        end
        
        t3 = t3 - [0.025, 0.025];
        if t3(1) < 0.0
           if c(3) == 0
              c(3) = 1; 
              p3 = x_animate(1:2,1)+yaw_rot(x_animate(9,1))*(p3_nom) + ...
                  yaw_rot(x_animate(9,1))*[x_dot;y_dot]*step_time*0.5;
              addpoints(foot3,p3(1,1),p3(2,1));
           else
              c(3) = 0;
              clearpoints(foot3);
           end
           t3(1) = t3(2);
           t3(2) = 2.0*step_time - 0.025;
        end
        
        t4 = t4 - [0.025, 0.025];
        if t4(1) < 0.0
           if c(4) == 0
              c(4) = 1; 
              p4 = x_animate(1:2,1)+yaw_rot(x_animate(9,1))*(p4_nom) + ...
                  yaw_rot(x_animate(9,1))*[x_dot;y_dot]*step_time*0.5;
              addpoints(foot4,p4(1,1),p4(2,1));
           else
              c(4) = 0;
              clearpoints(foot4);
           end
           t4(1) = t4(2);
           t4(2) = 2.0*step_time - 0.025;
        end
        
    end

    %% Callback function for detecting key presses
    function MyKeyDown(hObject, event, handles)
        key = get(hObject,'CurrentKey');
        KeyStatus = (strcmp(key, KeyNames) | KeyStatus);
    end
    function MyKeyUp(hObject, event, handles)
        key = get(hObject,'CurrentKey');
        KeyStatus = (~strcmp(key, KeyNames) & KeyStatus);
    end

end
