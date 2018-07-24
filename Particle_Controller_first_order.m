%% Header
%   Particle Dynamics and State Feedback Contro from Napora and Paley:
%   "Observer-Based Feedback Control For Stabilization od Collective
%   Motion"

%% Deklarations/Definitions
% Number of Particles/Vehicles
N = 4;
% Constant Speed
v = 0.1;
% Local Position vector of the Particles/Vehicles incl. starting position
r = [zeros(1,N);1:N];
% Local Speed orientation absolute
theta_abs = 1:N;
% Matrix of relative orientations
%theta_rel = zeros (N,N);
% Time-Derivation of Speed Orientation 
nu = zeros (1,N);
% Control Gain 
K = 1;
% Circle Radius
omega = 1;
% Timestepsize
dt = 0.01
% Simulation Time (s)
tmax = 15


%% Controller timesteps

for i = 0:dt:tmax
nu = zeros(1,N);
for ii = 1:N
    for jj = 1:N
        nu(ii) = nu(ii) - (K/N)*sin(theta_abs(jj)-theta_abs(ii));
    end
end 

theta_abs = nu * dt ;

%pause(dt);
end



