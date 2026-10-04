function [mu, Fx] = pacejka_tire(lambda, Fz, tire_params)
% PACEJKA_TIRE Calculates tire-road friction coefficient mu and longitudinal force Fx
% using the standard Pacejka Magic Formula.
%
% Inputs:
%   lambda      - Longitudinal slip ratio [0 to 1] (vector or scalar)
%   Fz          - Normal vertical load [N] (vector or scalar)
%   tire_params - Struct with fields B, C, D, E
%
% Outputs:
%   mu          - Friction coefficient [-]
%   Fx          - Longitudinal braking force [N] (positive opposes motion)

% Ensure non-negative slip magnitude for calculation
lam = abs(lambda);

% Magic formula calculation
B = tire_params.B;
C = tire_params.C;
D = tire_params.D;
E = tire_params.E;

arg = B .* lam - E .* (B .* lam - atan(B .* lam));
mu_mag = D .* sin(C .* atan(arg));

% Enforce physical bounds: friction coefficient cannot be negative for positive slip
mu_mag = max(0, mu_mag);

% Apply direction: braking force opposes vehicle motion
mu = sign(lambda) .* mu_mag;
Fx = mu .* Fz;

end
