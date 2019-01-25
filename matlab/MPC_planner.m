function MPC_planner()
    addpath('/home/romela/MPC/matlab')
% ========================================================================
% This is a high level planner that will pass footstep locations and a
% desired CM trajectory to a QP. 
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

    % Gait Parameters for a single leg
    swing_time = 0.2;
    stance_time = 0.22;
    
    % Timing parameters
    T_final = swing_time + stance_time; % final time of the entire plan
    dt = 0.03; % time step of the QP
    np = int64(T_final/dt);
    t = linspace(0.0, T_final, np);

    % State vector in the inertial frame
    x = zeros(12,1); % x = [position, euler angles, velocity, angular rate]
    x(3,1) = 0.5; % initial height
    current_state = x; % actual state
    xd = x; % desired state
    
    % velocities set by the user to determine the trajectory
    x_dot = 0.0;
    y_dot = 0.0;
    yaw_dot = 0.0;
        
    % Gait timing 
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
    
    c = [1, 1, 1, 1]; % indicates which foot is on the ground 1:on 0:off
    
    % Nominal foot positions
    p1_nom = [0.3; 0.2];
    p2_nom = [-0.3; 0.2];
    p3_nom = [-0.3; -0.2];
    p4_nom = [0.3; -0.2];
    
    % Current foot positions
    p1 = p1_nom;
    p2 = p2_nom;
    p3 = p3_nom;
    p4 = p4_nom;
    
    % Matrices for storing the entire trajectory
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
    
    x0 = zeros(13,1);
    x0(3,1) = 0.5;
    x0(13,1) = -9.81;
    
    x_ref = zeros(13*np,1);
    
    v1 = zeros(3*np,1);
    v2 = zeros(3*np,1);
    v3 = zeros(3*np,1);
    v4 = zeros(3*np,1);
    
    avg_yaw = 0.0;
    
    controller = MPCControllerGurobi_class(np,0.03);
    
    animate_count = 0;
    
    animator = Animation_class(x(1:3,1),p1,p2,p3,p4);

    %% Main loop for simulation
    while 1
        
        % Check the user inputs to update the desired velocities
        inputs = animator.GetUserInputs();
        [x_dot, y_dot, yaw_dot, quit] = CheckUserInputs(x_dot, y_dot, yaw_dot, inputs);
       
        if (quit == 1)
            close all;
            break
        end
        
        C(1,:) = ones(1,4);
        
        v1(1:3,1) = [p1; 0.0] - x0(1:3,1);
        v2(1:3,1) = [p2; 0.0] - x0(1:3,1);
        v3(1:3,1) = [p3; 0.0] - x0(1:3,1);
        v4(1:3,1) = [p4; 0.0] - x0(1:3,1);
        
        % Build the desired trajectory for 2 steps
        for i = 2:np  
            
            if i == 2
               % Set the initial conditions to the current state
               x_ref(1:13,1) = x0; 
               
               % No desired roll or pitch
               x_ref(3,1) = 0.5;
               x_ref(4:5,1) = [0.0; 0.0];
               x_ref(10:11,1) = [0.0; 0.0];
            end
            
            % Linear
            x_ref(13*(i-1)+7:13*(i-1)+8,1) = yaw_rot(x_ref(13*(i-2)+6))*[x_dot;y_dot];
            x_ref(13*(i-1)+1:13*(i-1)+2,1) = x_ref(13*(i-2)+1:13*(i-2)+2,1) + x_ref(13*(i-2)+7:13*(i-2)+8,1)*dt;
            x_ref(13*(i-1)+3,1) = 0.5;

            % Rotational
            x_ref(13*(i-1)+12,1) = yaw_dot;
            x_ref(13*(i-1)+6,1) = x_ref(13*(i-2)+6,1) + x_ref(13*(i-2)+12,1)*dt;
            
            x_ref(13*(i-1)+13,1) = -9.81;
            
            x = x_ref(13*(i-1)+1:13*(i-1)+13);
            
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
            
            v1(3*(i-1)+1:3*(i-1)+3,1) = [p1; 0.0] - x_ref(13*(i-1)+1:13*(i-1)+3,1);
            v2(3*(i-1)+1:3*(i-1)+3,1) = [p2; 0.0] - x_ref(13*(i-1)+1:13*(i-1)+3,1);
            v3(3*(i-1)+1:3*(i-1)+3,1) = [p3; 0.0] - x_ref(13*(i-1)+1:13*(i-1)+3,1);
            v4(3*(i-1)+1:3*(i-1)+3,1) = [p4; 0.0] - x_ref(13*(i-1)+1:13*(i-1)+3,1);
            
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
        [F, x0] = controller.update(x0, x_ref, avg_yaw, v1, v2, v3, v4, C);
        
        F
        
        x0
        
        % Set the current state to the last simulated state
        t1 = T1(2,:);
        t2 = T2(2,:);
        t3 = T3(2,:);
        t4 = T4(2,:);
        p1 = P1(:,2);
        p2 = P2(:,2);
        p3 = P3(:,2);
        p4 = P4(:,2);
        c = C(2,:);
        
        % Animation
        animate_count = animate_count + 1;
        if animate_count == 1
            animate_count = 0;
            animator.update(x0,p1,p2,p3,p4,c); 
        end
        
        % If the robot is going unstable stop the animation
        if abs(x0(4,1)) > 0.7
           break; 
        end
        
        if abs(x0(5,1)) > 0.7
           break; 
        end
        
    end

end

function [x_dot,y_dot,yaw_dot,quit] = CheckUserInputs(x_dot, y_dot, yaw_dot, inputs)
    KEY.FORWARD = 1;
    KEY.BACKWARD = 2;
    KEY.LEFT = 3;
    KEY.RIGHT = 4;
    KEY.ROT_LEFT = 5;
    KEY.ROT_RIGHT = 6;
    KEY.QUIT = 7;
    
    % Limits on velocity and acceleration parameters
    max_x_dot = 0.8;
    max_y_dot = 0.3;
    max_yaw_dot = 0.3;
    x_accel = 0.1;
    y_accel = 0.1;
    yaw_accel = 0.1;
    x_decel = 0.05;
    y_decel = 0.1;
    yaw_decel = 0.1;

    % Booleans determining if the robot should start decelerating
    y_decelerate = 1;
    x_decelerate = 1;
    yaw_decelerate = 1;
    quit = 0;

    % ============================
    % Check User Inputs
    %=============================
    if inputs(KEY.LEFT)
        y_decelerate = 0;
        y_dot = y_dot + y_accel;
        if y_dot > max_y_dot
            y_dot = max_y_dot;
        end
    end

    if inputs(KEY.RIGHT)
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

    if inputs(KEY.FORWARD)
        x_decelerate = 0;
        x_dot = x_dot + x_accel;
        if x_dot > max_x_dot
            x_dot = max_x_dot;
        end
    end

    if inputs(KEY.BACKWARD)
        x_decelerate = 0;
        x_dot = x_dot - x_accel;
        if x_dot < -max_x_dot
            x_dot = -max_x_dot;
        end
    end

    if x_decelerate
        if sign(x_dot)*x_dot < x_decel
           X = 0.0; 
        end
        if sign(x_dot)*x_dot > 0.0
           x_dot = x_dot - sign(x_dot)*x_decel;
        end
    end

    if inputs(KEY.ROT_LEFT)
        yaw_decelerate = 0;
        yaw_dot = yaw_dot + yaw_accel;
        if yaw_dot > max_yaw_dot
            yaw_dot = max_yaw_dot;
        end
    end

    if inputs(KEY.ROT_RIGHT)
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

    if inputs(KEY.QUIT)
        quit = 1;
    end
end
