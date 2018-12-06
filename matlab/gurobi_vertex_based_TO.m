function gurobi_vertex_based_TO()

    KeyStatus = false(1,7);    % Suppose you are using 6 keys in the game
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

    yaw_dot = 0.0;
    x_dot = 0.0;
    y_dot = 0.0;

    T_final = 0.5;
    dt = 0.01;
    np = T_final/dt;
    t = linspace(0.0, T_final, np);
    lin_accel = 2.0;
    rot_accel = 3.0;

    x = [0.0; 0.0; 0.5; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
    x_vel = 0.0;
    y_vel = 0.0;

    X = zeros(np,1);
    Y = zeros(np,1);

    f1 = figure('KeyPressFcn', @MyKeyDown, 'KeyReleaseFcn', @MyKeyUp);
    h = animatedline;
    axis([-10.0 10.0 -10.0 10.0]);
    addpoints(h,X',Y')
    
    n = 0;

    while 1
        y_decelerate = 1;
        x_decelerate = 1;
        yaw_decelerate = 1;

        if KeyStatus(KEY.FORWARD)
            y_decelerate = 0;
            y_dot = y_dot + 0.1;
            if y_dot > 1.0
                y_dot = 1.0;
            end
        end
        
        if KeyStatus(KEY.BACKWARD)
            y_decelerate = 0;
            y_dot = y_dot - 0.1;
            if y_dot < -1.0
                y_dot = -1.0;
            end
        end
        
        if y_decelerate
            if sign(y_dot)*y_dot < 0.5
               y_dot = 0.0; 
            end
            if sign(y_dot)*y_dot > 0.0
               y_dot = y_dot - sign(y_dot)*0.5;
            end
        end
        
        if KeyStatus(KEY.RIGHT)
            x_decelerate = 0;
            x_dot = x_dot + 0.1;
            if x_dot > 1.0
                x_dot = 1.0;
            end
        end
        
        if KeyStatus(KEY.LEFT)
            x_decelerate = 0;
            x_dot = x_dot - 0.1;
            if x_dot < -1.0
                x_dot = -1.0;
            end
        end
        
        if x_decelerate
            if sign(x_dot)*x_dot < 0.5
               x_dot = 0.0; 
            end
            if sign(x_dot)*x_dot > 0.0
               x_dot = x_dot - sign(x_dot)*0.5;
            end
        end
        
        if KeyStatus(KEY.ROT_LEFT)
            yaw_decelerate = 0;
            yaw_dot = yaw_dot + 0.2;
            if yaw_dot > 1.0
                yaw_dot = 1.0;
            end
        end
        
        if KeyStatus(KEY.ROT_RIGHT)
            yaw_decelerate = 0;
            yaw_dot = yaw_dot - 0.2;
            if yaw_dot < -1.0
                yaw_dot = -1.0;
            end
        end
        
        if yaw_decelerate
            if sign(yaw_dot)*yaw_dot < 0.5
               yaw_dot = 0.0; 
            end
            if sign(yaw_dot)*yaw_dot > 0.0
               yaw_dot = yaw_dot - sign(yaw_dot)*0.5;
            end
        end
        
        if KeyStatus(KEY.QUIT)
            close all;
           break 
        end
        
        drawnow
        
        % Coarsly integrate forward to find final state
        for i = 1:10
            % Linear
            x(4:5,1) = yaw_rot(x(9,1))*[x_dot;y_dot];
            x(1:2,1) = x(1:2,1) + x(4:5,1)*dt;

            % Rotational
            x(12,1) = yaw_dot;
            x(9,1) = x(9,1) + x(12,1)*dt;

            if i < 11
                X(n*10+i) = x(1,1);
                Y(n*10+i) = x(2,1);
            end
        end
        
        addpoints(h,X',Y')
        
        pause(0.1);
        
        n = n + 1;
        axis([-10.0+X(n), 10.0+X(n), -10.0+Y(n), 10.0+Y(n)]);
        if n > 4
            n = 0;
        end
    end

    function MyKeyDown(hObject, event, handles)
        key = get(hObject,'CurrentKey');
        KeyStatus = (strcmp(key, KeyNames) | KeyStatus);
    end
    function MyKeyUp(hObject, event, handles)
        key = get(hObject,'CurrentKey');
        KeyStatus = (~strcmp(key, KeyNames) & KeyStatus);
    end

end
