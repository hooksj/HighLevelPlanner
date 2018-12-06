function gurobi_vertex_based_TO()

    KeyStatus = false(1,6);    % Suppose you are using 6 keys in the game
    KeyNames = {'w', 'a','s', 'd', 'j', 'k'};
    KEY.FORWARD = 1;
    KEY.BACKWARD = 2;
    KEY.LEFT = 3;
    KEY.RIGHT = 4;
    KEY.NOTHING = 5;
    KEY.NOTHING2 = 6;

    % Function to create yaw rotation matrix
    yaw_rot = @(x) [cos(x), -sin(x); sin(x) cos(x);];

    prompt = 'yaw rate: ';
    yaw_dot = input(prompt);

    prompt = 'X vel: ';
    x_dot = input(prompt);

    prompt = 'Y vel: ';
    y_dot = input(prompt);

    T_final = 0.5;
    dt = 0.025;
    np = T_final/dt;
    t = linspace(0.0, T_final, np);
    lin_accel = 2.0;
    rot_accel = 3.0;

    x = [0.0; 0.0; 0.5; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0; 0.0];
    x_vel = 0.0;
    y_vel = 0.0;

    X = zeros(np,1);
    Y = zeros(np,1);

    % Coarsly integrate forward to find final state
    for i = 1:np
        % Linear
        if sign(x_dot)*x_dot > sign(x_dot)*x_vel
            x_vel = x_vel + sign(x_dot)*lin_accel*dt;
            if sign(x_dot)*x_dot < sign(x_dot)*x_vel
                x_vel = x_dot;
            end
        end

        if sign(y_dot)*y_dot > sign(y_dot)*y_vel
            y_vel = y_vel + sign(y_dot)*lin_accel*dt;
            if sign(y_dot)*y_dot < sign(y_dot)*y_vel
                y_vel = y_dot;
            end
        end
        x(4:5,1) = yaw_rot(x(9,1))*[x_vel;y_vel];
        x(1:2,1) = x(1:2,1) + x(4:5,1)*dt;


        % Rotational
        if sign(yaw_dot)*yaw_dot > sign(yaw_dot)*x(12,1)
            x(12,1) = x(12,1) + sign(yaw_dot)*rot_accel*dt;
            if sign(yaw_dot)*yaw_dot < sign(yaw_dot)*x(12,1)
                x(12,1) = yaw_dot;
            end
        end
        x(9,1) = x(9,1) + x(12,1)*dt;

        X(i) = x(1,1);
        Y(i) = x(2,1);
    end

    f1 = figure('KeyPressFcn', @MyKeyDown, 'KeyReleaseFcn', @MyKeyUp);
    h = animatedline;
    addpoints(h,X',Y')

    value = 0;

    while 1
        if KeyStatus(KEY.FORWARD)  % If left key is pressed
                value = value + 1
        end
        drawnow

        if value == 100
            break
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
