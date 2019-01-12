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
    % Function to create yaw rotation matrix
    yaw_rot = @(x) [cos(x), -sin(x); sin(x) cos(x);];

    % Look ahead timing parameters
    swing_time = 0.2;
    stance_time = 0.22;
    T_final = swing_time + stance_time;
    dt = 0.001;
    np = int64(T_final/dt);
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
    t1 = [0.0, swing_time];
    t2 = [stance_time-(stance_time-swing_time)*0.5, swing_time+stance_time-(stance_time-swing_time)*0.5];
    t3 = [0.0, swing_time];
    t4 = [stance_time-(stance_time-swing_time)*0.5, swing_time+stance_time-(stance_time-swing_time)*0.5];
    
%     t1 = [0.0, step_time];
%     t2 = [2.0*step_time, 3.0*step_time];
%     t3 = [step_time, 2.0*step_time];
%     t4 = [3.0*step_time, 4.0*step_time];
    
%     t1 = [step_time, 2.0*step_time];
%     t2 = [3.0*step_time, 4.0*step_time];
%     t3 = [0.0, step_time];
%     t4 = [2.0*step_time, 3.0*step_time];
    
    c = [1, 1, 1, 1];
    
    p1_nom = [0.3; 0.2];
    p2_nom = [-0.3; 0.2];
    p3_nom = [-0.3; -0.2];
    p4_nom = [0.3; -0.2];
    
    p1 = p1_nom;
    p2 = p2_nom;
    p3 = p3_nom;
    p4 = p4_nom;
    
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
    
    animator = Animation_class(x(1:3,1),p1,p2,p3,p4);

    %% Main loop for simulation
    while 1
        
        % Check the user inputs to update the desired velocities
        [x_dot, y_dot, yaw_dot, quit] = animator.CheckUserInputs();
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
                      yaw_rot(x(6,1))*[x_dot;y_dot]*stance_time*0.5;
               else
                  c(1) = 0;
               end
               t1(1) = t1(2);
               t1(2) = swing_time+stance_time - dt;
            end

            t2 = t2 - [dt, dt];
            if t2(1) < 0.0
               if c(2) == 0
                  c(2) = 1; 
                  p2 = x(1:2,1)+yaw_rot(x(6,1))*(p2_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*stance_time*0.5;
               else
                  c(2) = 0;
               end
               t2(1) = t2(2);
               t2(2) = swing_time+stance_time - dt;
            end

            t3 = t3 - [dt, dt];
            if t3(1) < 0.0
               if c(3) == 0
                  c(3) = 1; 
                  p3 = x(1:2,1)+yaw_rot(x(6,1))*(p3_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*stance_time*0.5;
               else
                  c(3) = 0;
               end
               t3(1) = t3(2);
               t3(2) = swing_time+stance_time - dt;
            end

            t4 = t4 - [dt, dt];
            if t4(1) < 0.0
               if c(4) == 0
                  c(4) = 1; 
                  p4 = x(1:2,1)+yaw_rot(x(6,1))*(p4_nom) + ...
                      yaw_rot(x(6,1))*[x_dot;y_dot]*stance_time*0.5;
               else
                  c(4) = 0;
               end
               t4(1) = t4(2);
               t4(2) = swing_time+stance_time - dt;
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
        
        [F, current_state, X(:,n)];
        
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
        if animate_count == 1
            animate_count = 0;
            animator.update(x,p1,p2,p3,p4,c); 
        end
        
        if abs(x(4,1)) > 0.7
           break; 
        end
        
        if abs(x(5,1)) > 0.7
           break; 
        end
        
    end

end
