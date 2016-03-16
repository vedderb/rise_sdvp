%******************************************************************************************
% Magnetometer calibration, ellipsoid fit utility
%
% (c) Davide Gironi, 2012
% http://davidegironi.blogspot.it/
%
% References:
%   - Magnetometer Calibration Skript for Razor AHRS v1.4.1
%     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
%   - Yury Petrov ellipsoid_fit
%     http://www.mathworks.com/matlabcentral/fileexchange/24693-ellipsoid-fit
%
% Released under 3 GNU General Public License
%******************************************************************************************

% read magnetometer data
[M] = dlmread('magnpoints.txt', '\t');

% get values
x = M(:,1);
y = M(:,2);
z = M(:,3);

% do ellipsoid fitting
[e_center, e_radii, e_eigenvecs, e_algebraic] = ellipsoid_fit([x, y, z]);

% compensate distorted magnetometer data
% e_eigenvecs is an orthogonal matrix, so ' can be used instead of inv()
S = [x - e_center(1), y - e_center(2), z - e_center(3)]'; % translate and make array
scale = inv([e_radii(1) 0 0; 0 e_radii(2) 0; 0 0 e_radii(3)]) * min(e_radii); % scaling matrix
map = e_eigenvecs'; % transformation matrix to map ellipsoid axes to coordinate system axes
invmap = e_eigenvecs; % inverse of above
comp = invmap * scale * map;
S = comp * S; % do compensation

% output info
fprintf( 'Ellipsoid center     :\n                   %.3g %.3g %.3g\n', e_center );
fprintf( 'Ellipsoid radii      :\n                   %.3g %.3g %.3g\n', e_radii );
fprintf( 'Ellipsoid evecs      :\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n', e_eigenvecs );
fprintf( 'Ellpisoid comp evecs :\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n                   %.6g %.6g %.6g\n', comp);

res = fopen('res.txt', 'w');
fprintf(res, 'main_config.mag_cal_cx = %.6g;\n', e_center(1));
fprintf(res, 'main_config.mag_cal_cy = %.6g;\n', e_center(2));
fprintf(res, 'main_config.mag_cal_cz = %.6g;\n\n', e_center(3));

fprintf(res, 'main_config.mag_cal_xx = %.6g;\n', comp(1));
fprintf(res, 'main_config.mag_cal_xy = %.6g;\n', comp(2));
fprintf(res, 'main_config.mag_cal_xz = %.6g;\n\n', comp(3));

fprintf(res, 'main_config.mag_cal_yx = %.6g;\n', comp(4));
fprintf(res, 'main_config.mag_cal_yy = %.6g;\n', comp(5));
fprintf(res, 'main_config.mag_cal_yz = %.6g;\n\n', comp(6));

fprintf(res, 'main_config.mag_cal_zx = %.6g;\n', comp(7));
fprintf(res, 'main_config.mag_cal_zy = %.6g;\n', comp(8));
fprintf(res, 'main_config.mag_cal_zz = %.6g;\n\n', comp(9));
fclose(res);

% draw data
figure;
hold on;
plot3( x, y, z, '.r' ); % original magnetometer data
plot3(S(1,:), S(2,:), S(3,:), 'b.'); % compensated data
view( -70, 40 );
%axis vis3d;
axis equal;

% draw ellipsoid fit
figure;
hold on;
plot3( x, y, z, '.r' );
maxd = max(e_radii);
step = maxd / 50;
[xp, yp, zp] = meshgrid(-maxd:step:maxd + e_center(1), -maxd:step:maxd + e_center(2), -maxd:step:maxd + e_center(3));
Ellipsoid = e_algebraic(1) *xp.*xp +   e_algebraic(2) * yp.*yp + e_algebraic(3)   * zp.*zp + ...
          2*e_algebraic(4) *xp.*yp + 2*e_algebraic(5) * xp.*zp + 2*e_algebraic(6) * yp.*zp + ...
          2*e_algebraic(7) *xp     + 2*e_algebraic(8) * yp     + 2*e_algebraic(9) * zp;
          
%p = patch(isosurface(xp, yp, zp, Ellipsoid, 1));
%set(p, 'FaceColor', 'g', 'EdgeColor', 'none');
%alpha(0.5);
%view( -70, 40 );
%axis vis3d;
%axis equal;
%camlight;
%lighting phong;
