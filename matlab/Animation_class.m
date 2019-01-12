classdef Animation_class < handle
    properties
        
        KeyStatus = false(1,7);
        KeyNames = {'w', 's','a', 'd', 'j', 'k', 'q'};
        
        % Limits on velocity and acceleration parameters
        max_x_dot = 0.5;
        max_y_dot = 0.2;
        max_yaw_dot = 0.3;
        x_accel = 0.1;
        y_accel = 0.1;
        yaw_accel = 0.1;
        x_decel = 0.01;
        y_decel = 0.01;
        yaw_decel = 0.05;
        
        x_dot = 0.0;
        y_dot = 0.0;
        yaw_dot = 0.0;
        
        % Body dimensions if the CM is at (0,0,0)
        body_points = [0.15  0.0 -0.05;...
                       0.0  0.15 -0.05;...
                      -0.15  0.0 -0.05;...
                       0.0 -0.15 -0.05;...
                       0.15  0.0  0.05;...
                       0.0  0.15  0.05;...
                      -0.15  0.0  0.05;...
                       0.0 -0.15  0.05];
           
        % Maps which body vertices go with which face of the cube    
        faces = [1 2 6 5;2 3 7 6;3 4 8 7;4 1 5 8;1 2 3 4;5 6 7 8];
        
        body % the object use to plot the body
        foot1 
        foot2
        foot3
        foot4
        
        ground % the object that is the ground
        
        fig % the figure object
        
    end
    
    methods
        function obj = Animation_class(cm, p1, p2, p3, p4) 
            points = obj.body_points + [cm'; cm'; cm'; cm'; cm'; cm'; cm'; cm'];
            obj.fig = figure('KeyPressFcn', @MyKeyDown, 'KeyReleaseFcn', @MyKeyUp);
            obj.fig.UserData = false(1,7);
            set(obj.fig, 'Position', get(0,'Screensize'));
            obj.body = patch('Vertices',points,'Faces',obj.faces,...
                             'FaceVertexCData',hsv(1),'FaceColor','flat');
            hold on
            [X,Y] = meshgrid(-100:100,-100:100);
            Z = 0*X;
            C = zeros(size(X,1),size(X,2),3);
            C(:,:,2) = ones(size(X));
            obj.ground = surf(X,Y,Z,C);
            
            [x,y,z] = sphere();
            obj.foot1 = surf(x*0.025+p1(1,1), y*0.025+p1(2,1), z*0.025+0.0); 
            obj.foot2 = surf(x*0.025+p2(1,1), y*0.025+p2(2,1), z*0.025+0.0); 
            obj.foot3 = surf(x*0.025+p3(1,1), y*0.025+p3(2,1), z*0.025+0.0); 
            obj.foot4 = surf(x*0.025+p4(1,1), y*0.025+p4(2,1), z*0.025+0.0); 
            
            view(3)
            axis([-1.0 1.0 -1.0 1.0 0.0 2.0])
            axis vis3d       
        end
        
        function update(obj,x,p1,p2,p3,p4,c)
            R = obj.rot(x(4:6,1));
            points = zeros(size(obj.body_points));
            for i = 1:length(obj.body_points)
                points(i,:) = x(1:3,1)' + obj.body_points(i,:)*R';
            end
            set(obj.body, 'Vertices', points);
            
            [rx,ry,rz] = sphere();
            set(obj.foot1, 'XData', rx*0.025+p1(1,1));
            set(obj.foot1, 'YData', ry*0.025+p1(2,1));
            set(obj.foot1, 'ZData', rz*0.025+0.0);
            set(obj.foot1, 'FaceAlpha', c(1));
            set(obj.foot1, 'EdgeAlpha', c(1));
            
            set(obj.foot2, 'XData', rx*0.025+p2(1,1));
            set(obj.foot2, 'YData', ry*0.025+p2(2,1));
            set(obj.foot2, 'ZData', rz*0.025+0.0);
            set(obj.foot2, 'FaceAlpha', c(2));
            set(obj.foot2, 'EdgeAlpha', c(2));
            
            set(obj.foot3, 'XData', rx*0.025+p3(1,1));
            set(obj.foot3, 'YData', ry*0.025+p3(2,1));
            set(obj.foot3, 'ZData', rz*0.025+0.0);
            set(obj.foot3, 'FaceAlpha', c(3));
            set(obj.foot3, 'EdgeAlpha', c(3));
            
            set(obj.foot4, 'XData', rx*0.025+p4(1,1));
            set(obj.foot4, 'YData', ry*0.025+p4(2,1));
            set(obj.foot4, 'ZData', rz*0.025+0.0);
            set(obj.foot4, 'FaceAlpha', c(4));
            set(obj.foot4, 'EdgeAlpha', c(4));
            
            axis([-1.0+x(1,1) 1.0+x(1,1) -1.0+x(2,1) 1.0+x(2,1) 0.0 2.0])
            drawnow;
        end
        
        function R = rot(obj, x)
            R = [cos(x(3))*cos(x(2)) cos(x(3))*sin(x(2))*sin(x(1))-sin(x(3))*cos(x(1)) cos(x(3))*sin(x(2))*cos(x(1))+sin(x(3))*sin(x(1));
                 sin(x(3))*cos(x(2)) sin(x(3))*sin(x(2))*sin(x(1))+cos(x(3))*cos(x(1)) sin(x(3))*sin(x(2))*cos(x(1))-cos(x(3))*sin(x(1));
                 -sin(x(2))                        cos(x(2))*sin(x(1))                               cos(x(2))*cos(x(1))                ];
        end
        
        function [X,Y,YAW,quit] = CheckUserInputs(obj)
            KEY.FORWARD = 1;
            KEY.BACKWARD = 2;
            KEY.LEFT = 3;
            KEY.RIGHT = 4;
            KEY.ROT_LEFT = 5;
            KEY.ROT_RIGHT = 6;
            KEY.QUIT = 7;
            
            % Booleans determining if the robot should start decelerating
            y_decelerate = 1;
            x_decelerate = 1;
            yaw_decelerate = 1;
            quit = 0;
            
            X = 0.0;
            Y = 0.0;
            YAW = 0.0;

            % ============================
            % Check User Inputs
            %=============================
            if obj.fig.UserData(KEY.RIGHT)
                y_decelerate = 0;
                Y = obj.y_dot + obj.y_accel;
                if Y > obj.max_y_dot
                    Y = obj.max_y_dot;
                end
            end

            if obj.fig.UserData(KEY.LEFT)
                y_decelerate = 0;
                Y = obj.y_dot - obj.y_accel;
                if Y < -obj.max_y_dot
                    Y = -obj.max_y_dot;
                end
            end

            if y_decelerate
                if sign(obj.y_dot)*obj.y_dot < obj.y_decel
                   Y = 0.0; 
                end
                if sign(Y)*Y > 0.0
                   Y = obj.y_dot - sign(obj.y_dot)*obj.y_decel;
                end
            end

            if obj.fig.UserData(KEY.FORWARD)
                x_decelerate = 0;
                X = obj.x_dot + obj.x_accel;
                if X > obj.max_x_dot
                    X = obj.max_x_dot;
                end
            end

            if obj.fig.UserData(KEY.BACKWARD)
                x_decelerate = 0;
                X = obj.x_dot - obj.x_accel;
                if X < -obj.max_x_dot
                    X = -obj.max_x_dot;
                end
            end

            if x_decelerate
                if sign(obj.x_dot)*obj.x_dot < obj.x_decel
                   X = 0.0; 
                end
                if sign(obj.x_dot)*obj.x_dot > 0.0
                   X = obj.x_dot - sign(obj.x_dot)*obj.x_decel;
                end
            end

            if obj.fig.UserData(KEY.ROT_LEFT)
                yaw_decelerate = 0;
                YAW = obj.yaw_dot + obj.yaw_accel;
                if YAW > obj.max_yaw_dot
                    YAW = obj.max_yaw_dot;
                end
            end

            if obj.fig.UserData(KEY.ROT_RIGHT)
                yaw_decelerate = 0;
                YAW = obj.yaw_dot - obj.yaw_accel;
                if YAW < -obj.max_yaw_dot
                    YAW = -obj.max_yaw_dot;
                end
            end

            if yaw_decelerate
                if sign(obj.yaw_dot)*obj.yaw_dot < obj.yaw_decel
                   YAW = 0.0; 
                end
                if sign(obj.yaw_dot)*obj.yaw_dot > 0.0
                   YAW = obj.yaw_dot - sign(obj.yaw_dot)*obj.yaw_decel;
                end
            end
            
            obj.x_dot = X;
            obj.y_dot = Y;
            obj.yaw_dot = YAW;

            if obj.fig.UserData(KEY.QUIT)
                quit = 1;
            end
        end
        
        function obj = set.x_dot(obj, num)
           obj.x_dot = num; 
        end
        
        function obj = set.y_dot(obj, num)
           obj.y_dot = num; 
        end
        
        function obj = set.yaw_dot(obj, num)
           obj.yaw_dot = num; 
        end
        
    end
    
end

%% Callback function for detecting key presses
function MyKeyDown(hObject, event, handles)
    KeyNames = {'w', 's','a', 'd', 'j', 'k', 'q'};
    KEY.FORWARD = 1;
    KEY.BACKWARD = 2;
    KEY.LEFT = 3;
    KEY.RIGHT = 4;
    KEY.ROT_LEFT = 5;
    KEY.ROT_RIGHT = 6;
    KEY.QUIT = 7;
    key = get(hObject,'CurrentKey');
    hObject.UserData = (strcmp(key, KeyNames) | hObject.UserData);
end

function MyKeyUp(hObject, event, handles)
    KeyNames = {'w', 's','a', 'd', 'j', 'k', 'q'};
    KEY.FORWARD = 1;
    KEY.BACKWARD = 2;
    KEY.LEFT = 3;
    KEY.RIGHT = 4;
    KEY.ROT_LEFT = 5;
    KEY.ROT_RIGHT = 6;
    KEY.QUIT = 7;
    key = get(hObject,'CurrentKey');
    hObject.UserData = (~strcmp(key, KeyNames) & hObject.UserData);
end
        
        
            