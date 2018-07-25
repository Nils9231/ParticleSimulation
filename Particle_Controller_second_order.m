%% 
%   Particle Dynamics and State Feedback Contro from Napora and Paley:
%   "Observer-Based Feedback Control For Stabilization od Collective
%   Motion"
%
%   Second order Steering

%% Deklarations/Definitions
% Timestepsize
dt = 0.01;
% Simulation Time (s)
tmax = 15;
% Number of Particles/Vehicles
N = 8;
% Constant Speed
v = 1;
% Local Position vector of the Particles/Vehicles incl. starting position
r = zeros(2*tmax/dt,N);
r(1,:) = zeros(1,N);
r(2,:) = 1:N;
% Local Speed orientation absolute
theta_abs = zeros(tmax/dt+1,N);
theta_abs(1,:) = 1:N;
% Matrix of relative orientations
%theta_rel = zeros (N,N);
% Angular Velocity Vector
omega = zeros(tmax/dt,N);
% Angular acceleration Vector
u = zeros (tmax/dt,N);
% Position differences
drx = zeros(1,N);
dry = zeros(1,N);
% Time-Derivation of Speed Orientation 
nu = zeros (1,N);
% Control Gain 
K = -1;
%   K<0 for straight-line Motion (parrallel trajectories)
%   K>0 for balanced Motion (Sum of all velocities equal zero)
% Parallel Controll Gain
Kp = 1;




%% timesteps

clf;                                    
axis square;
axis([-20 20 -20 20]);
grid on;
hold on;                               
title('Motion animation');
xlabel('x (m)');  
ylabel('y (m)');
for ii = 1:(tmax/dt)
    nu = zeros(1,N);
    for jj = 1:N
        for kk = 1:N
            nu(jj) = nu(jj) - (K/N)*sin(theta_abs(ii,kk)-theta_abs(ii,jj));
        end
    end 
    u(ii+1,:)= u(ii,:)+ Kp*(nu-omega(ii,:));
    
    omega(ii+1,:) = omega(ii,:) + u(ii+1,:)*dt;
    
    theta_abs(ii+1,:) = theta_abs(ii,:)+ omega(ii+1,:) * dt ;

    drx = cos(theta_abs(ii,:))*v*dt;
    dry = sin(theta_abs(ii,:))*v*dt;
    
    r(2*(ii+1)-1,:)= r(2*ii-1,:)+drx;
    r(2*(ii+1),:)= r(2*ii,:)+dry;
    
    plot(r(2*ii-1,:),r(2*ii,:),'b.');
    
    pause(dt);
end

hold off;


