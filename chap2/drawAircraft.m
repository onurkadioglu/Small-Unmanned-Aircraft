function drawAircraft(uu)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);       % inertial Down postion
    u        = uu(4);       % linear velocity in x-direction
    v        = uu(5);       % linear velocity in y-diretion
    w        = uu(6);       % linear velocity in z-direction
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);      % roll rate
    q        = uu(11);      % pitch rate     
    r        = uu(12);      % yaw rate    
    t        = uu(13);      % time

    % define persistent variables 
    persistent aircraft_handle;
    persistent Vertices
    persistent Faces
    persistent facecolors
    
    % first time function is called, initialize plot and persistent vars
    if t==0
        figure(1), clf
        [Vertices,Faces,facecolors] = defineAircraftBody;
        aircraft_handle = drawAircraftBody(Vertices,Faces,facecolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-400,400,-400,400,-400,400]);
        grid
        hold on
        
    % at every other time step, redraw base and rod
    else 
        drawAircraftBody(Vertices,Faces,facecolors,...
                           pn,pe,pd,phi,theta,psi,...
                           aircraft_handle);
    end
end

  
%=======================================================================
% drawAircraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawAircraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V, phi, theta, psi);  % rotate aircraft
  V = translate(V, pn, pe, pd);  % translate aircraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = R*V;
  
  if isempty(handle)
  handle = patch('Vertices', V', 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V','Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi)

  % define rotation matrix (right handed)
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
      
  R = R_roll*R_pitch*R_yaw;  

  % rotate vertices
  XYZ = R*XYZ;
  
end
% end rotateVert

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)

  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
  
end

% end translate


%=======================================================================
% defineAircraftBody
%=======================================================================
        function [V, F, colors] = defineAircraftBody
            % Define the vertices (physical location of vertices)
            V = [...
                30     0    0;... % point 1
                20     10   -10;... % point 2
                20    -10   -10;... % point 3
                20    -10    10;... % point 4
                20     10    10;... % point 5
               -90     0    0;... % point 6
                0     50    0;... % point 7
               -20     50    0;... % point 8
               -20    -50    0;... % point 9
                0    -50    0;... % point 10
               -65   30    0;... % point 11
               -90     30    0;... % point 12
               -90    -30    0;... % point 13
               -65  -30    0;... % point 14
               -65   0    0;... % point 15
               -90     0 -25;... % point 16
            ]';

            % define faces as a list of vertices numbered above
            F = [...
                1, 1, 2, 5;...     % nose right
                1, 1, 3, 4;...     % nose left
                1, 1, 2, 3;...     % nose top
                1, 1, 4, 5;...     % nose bottom
                2, 5, 6, 6;...     % fuselage right
                3, 4, 6, 6;...     % fuselage left
                2, 3, 6, 6;...     % fuselage top
                4, 5, 6, 6;...     % fuselage bottom
                7, 8, 9, 10;...    % wing
                11, 12, 13, 14;... % horizontal tail
                6, 15, 16, 6;...   % vertical tail
              ];

            % define colors for each face    
            myred = [1, 0, 0];
            mygreen = [0, 1, 0];
            myblue = [0, 0, 1];
            myyellow = [1, 1, 0];
            mycyan = [0, 1, 1];

            colors = [...
                 mygreen;...      % nose right
                 mygreen;...      % nose left
                 mygreen;...      % nose top
                 mygreen;...      % nose bottom
                 myred;...        % fuselage right
                 myred;...        % fuselage left
                 myred;...        % fuselage top
                 myred;...        % fuselage bottom
                 myblue;...       % wing
                 myyellow;...     % horizontal tail
                 mygreen;...      % vertical tail
                ];
        end