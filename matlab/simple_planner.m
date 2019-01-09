function simple_planner()
    addpath('/home/romela/Deadbeat_Controller/Gurobi')
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
    max_x_dot = 0.2;
    max_y_dot = 0.2;
    max_yaw_dot = 1.5;
    x_accel = 0.01;
    y_accel = 0.01;
    yaw_accel = 0.2;
    x_decel = 0.01;
    y_decel = 0.01;
    yaw_decel = 0.5;

    % Look ahead timing parameters
    step_time = 0.2;
    T_final = 2*step_time;
    dt = 0.001;
    np = T_final/dt;
    t = linspace(0.0, T_final, np);

    % State vector in the inertial frame
    % x = [position, euler angles, velocity, angular rate]
    x = zeros(12,1);
    x(3,1) = 0.5; 
    current_state = x;
    xd = x;
    
    % Body dimensions
    body = [0.1, -0.1, -0.1, 0.1;
            0.2, 0.2, -0.2, -0.2];
        
    % Foot parameters
%     t1 = [0.0, step_time];
%     t2 = [step_time, 2.0*step_time];
%     t3 = [0.0, step_time];
%     t4 = [step_time, 2.0*step_time];
    
    t1 = [0.0, step_time];
    t2 = [2.0*step_time, 3.0*step_time];
    t3 = [step_time, 2.0*step_time];
    t4 = [3.0*step_time, 4.0*step_time];
    
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
    block = patch([2, 1, 1, 2], [2, 2, 1, 1], 'black');
    block2 = patch([2, 1, 1, 2], [4, 4, 3, 3], 'black');
    foot1 = animatedline('Marker','x');
    foot2 = animatedline('Marker','x');
    foot3 = animatedline('Marker','x');
    foot4 = animatedline('Marker','x');
    addpoints(foot1, p1(1,1), p1(2,1));
    addpoints(foot2, p2(1,1), p2(2,1));
    addpoints(foot3, p3(1,1), p3(2,1));
    addpoints(foot4, p4(1,1), p4(2,1));
    
    x_animate = x;
    t1_animate = t1;
    t2_animate = t2;
    t3_animate = t3;
    t4_animate = t4;
    c_animate = c;
    
    X = zeros(length(x),np);
    F1 = zeros(3,np);
    F2 = zeros(3,np);
    F3 = zeros(3,np);
    F4 = zeros(3,np);
    P1 = zeros(2,np);
    P2 = zeros(2,np);
    P3 = zeros(2,np);
    P4 = zeros(2,np);
    T1 = zeros(np,2);
    T2 = zeros(np,2);
    T3 = zeros(np,2);
    T4 = zeros(np,2);
    C = zeros(np,4);
    
    controller = DeadbeatControllerGurobi_class;
    
    animate_count = 0;

    %% Main loop for simulation
    while 1
        
        % Check the user inputs to update the desired velocities
        quit = CheckUserInputs();
        if (quit == 1)
            close all;
            break
        end

        
        % Build the desired trajectory for 2 steps
        for i = 1:np            
            % Linear
            xd(7:8,1) = yaw_rot(xd(6,1))*[x_dot;y_dot];
            xd(1:2,1) = xd(1:2,1) + xd(7:8,1)*dt;

            % Rotational
            xd(12,1) = yaw_dot;
            xd(6,1) = xd(6,1) + xd(12,1)*dt;
            
            X(:,i) = xd;
            
            % Determine the footstep locations
            t1 = t1 - [dt, dt];
            if t1(1) < 0.0
               if c(1) == 0
                  c(1) = 1; 
                  p1 = x(1:2,1)+yaw_rot(x(6,1))*(p1_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*step_time*0.5;
               else
                  c(1) = 0;
               end
               t1(1) = t1(2);
               t1(2) = 4.0*step_time - dt;
            end

            t2 = t2 - [dt, dt];
            if t2(1) < 0.0
               if c(2) == 0
                  c(2) = 1; 
                  p2 = x(1:2,1)+yaw_rot(x(6,1))*(p2_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*step_time*0.5;
               else
                  c(2) = 0;
               end
               t2(1) = t2(2);
               t2(2) = 4.0*step_time - dt;
            end

            t3 = t3 - [dt, dt];
            if t3(1) < 0.0
               if c(3) == 0
                  c(3) = 1; 
                  p3 = x(1:2,1)+yaw_rot(x(6,1))*(p3_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*step_time*0.5;
               else
                  c(3) = 0;
               end
               t3(1) = t3(2);
               t3(2) = 4.0*step_time - dt;
            end

            t4 = t4 - [dt, dt];
            if t4(1) < 0.0
               if c(4) == 0
                  c(4) = 1; 
                  p4 = x(1:2,1)+yaw_rot(x(6,1))*(p4_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*step_time*0.5;
               else
                  c(4) = 0;
               end
               t4(1) = t4(2);
               t4(2) = 4.0*step_time - dt;
            end
            
            % Store trajectory
            P1(:,i) = p1;
            P2(:,i) = p2;
            P3(:,i) = p3;
            P4(:,i) = p4;
            T1(i,:) = t1;
            T2(i,:) = t2;
            T3(i,:) = t3;
            T4(i,:) = t4;
            C(i,:) = c;
        end
        
        % Simulate dynamics
        for n = 2:26
            [F, current_state] = controller.update(current_state, [P1(:,n);0.0], ...
                                                   [P2(:,n);0.0], [P3(:,n);0.0], ...
                                                   [P4(:,n);0.0], C(n,:), X(:,n), dt);
        end
        
        [F, current_state, X(:,n)]
        
        % Set the current state to the last simulated state
        x = current_state;
        xd = X(:,26);
        t1 = T1(26,:);
        t2 = T2(26,:);
        t3 = T3(26,:);
        t4 = T4(26,:);
        p1 = P1(:,26);
        p2 = P2(:,26);
        p3 = P3(:,26);
        p4 = P4(:,26);
        c = C(26,:);
        
        % Animation
        animate_count = animate_count + 1;
        if animate_count == 4
            animate_count = 0;
            x_animate = x;
            clearpoints(foot1);
            clearpoints(foot2);
            clearpoints(foot3);
            clearpoints(foot4);
            if c(1) == 1
                addpoints(foot1,p1(1,1),p1(2,1));
            end

            if c(2) == 1
                addpoints(foot2,p2(1,1),p2(2,1));
            end

            if c(3) == 1
                addpoints(foot3,p3(1,1),p3(2,1));
            end

            if c(4) == 1
                addpoints(foot4,p4(1,1),p4(2,1));
            end
            animate();   
        end
        
        if abs(x(4,1)) > 0.7
           break; 
        end
        
        if abs(x(5,1)) > 0.7
           break; 
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

    %% Animate function
    function animate()
        points = zeros(2,4);
        for j = 1:4
            points(:,j) = x_animate(1:2,1)+yaw_rot(x_animate(6,1))*(body(:,j));
        end
        set(H, 'XData', points(1,:));
        set(H, 'YData', points(2,:));
        axis([-1.5+x_animate(1,1), 1.5+x_animate(1,1),...
            -1.5+x_animate(2,1), 1.5+x_animate(2,1)]);
        drawnow;
        
        %pause(0.1);
        
    end

    %% Check user inputs function
    function quit = CheckUserInputs()
        % Booleans determining if the robot should start decelerating
        y_decelerate = 1;
        x_decelerate = 1;
        yaw_decelerate = 1;
        quit = 0;

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
            quit = 1;
        end
    end

end
