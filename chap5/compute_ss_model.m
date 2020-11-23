% x_trim is the trimmed state,
% u_trim is the trimmed input
  
  
[A,B,C,D]=linmod('mavsim_trim',x_trim,u_trim);

lat_state = [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0; % v
             0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0; % p
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1; % r
             0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; % phi
             0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0] % psi

lat_input = [0, 1, 0, 0; % delta - a
             0, 0, 1, 0] % delta - r

lon_state = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; % u
             0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0; % w
             0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; % q
             0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; % theta
             0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0]% h

lon_input = [1, 0, 0, 0; %delta - e
             0, 0, 0, 1];%delta - t

A_lat = lat_state*A*lat_state';
B_lat = lat_state*B*lat_input';

A_lon = lon_state*A*lon_state';
B_lon = lon_state*B*lon_input';
  