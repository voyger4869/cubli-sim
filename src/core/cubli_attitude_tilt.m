function U = cubli_attitude_tilt(Rl,n)
%CUBLI_ATTITUDE_TILT World direction of a body-fixed vector, from the R log.
%
%   U = cubli_attitude_tilt(Rl,n)   Rl is 9-by-N (the logged rotation matrix,
%                                   column-major per sample), n is 3-by-1.
%
% Returns U, 3-by-N, with U(:,k) = Rk*n -- the attitude of the body vector n
% expressed in world coordinates.
%
% This exists because the obvious shortcut is wrong and quietly so. Writing
%
%   reshape(Rl,3,[])        % WRONG for a 9-by-N log
%
% flattens ACROSS samples instead of reshaping each 9-element sample, so it
% returns a smooth-looking but meaningless signal -- it produced a vertex tilt
% oscillating through +/-45 deg while com_z sat exactly at the balanced height,
% which is impossible for a rigid body resting on one corner. The shapes are
% compatible, so nothing errors; only the physics betrays it.
%
% For the vertex, n = (1,1,1)/sqrt(3) is the body diagonal that points up at
% the balance attitude, and U = (theta_y, -theta_x, 1) for small tilts.

assert(size(Rl,1) == 9,'CubliClean:AttitudeLog','R log must have 9 components.');
N = size(Rl,2);
Rm = reshape(Rl,3,3,N);
U = squeeze(sum(Rm.*reshape(n,1,3),2));
assert(isequal(size(U),[3 N]),'CubliClean:AttitudeShape', ...
    'Expected 3-by-%d output, got %s.',N,mat2str(size(U)));
end
