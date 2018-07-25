%% 
%   Particle Dynamics and State Feedback Contro from Napora and Paley:
%   "Observer-Based Feedback Control For Stabilization od Collective
%   Motion"
%
%   first order steering

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
% Position differences
rx = zeros(1,N);
ry = zeros(1,N);
% Time-Derivation of Speed Orientation 
nu = zeros (1,N);
% Control Gain 
K = -1;
%   K<0 for straight-line Motion (parrallel trajectories)
%   K>0 for balanced Motion (Sum of all velocities equal zero)


% Circular Motion?
circular_motion = 1; % 1 Yes 0 No



%% timesteps

if circular_motion == 0

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
                nu(jj) = nu(jj) + sin(theta_abs(ii,kk)-theta_abs(ii,jj));
            end
        end 
        nu = - (K/N)*nu;
        theta_abs(ii+1,:) = theta_abs(ii,:)+ nu * dt ;

        rx = cos(theta_abs(ii,:))*v*dt;
        ry = sin(theta_abs(ii,:))*v*dt;

        r(2*(ii+1)-1,:)= r(2*ii-1,:)+rx;
        r(2*(ii+1),:)= r(2*ii,:)+ry;

        plot(r(2*ii-1,:),r(2*ii,:),'b.');

        pause(dt);
    end

    hold off;
elseif circular_motion == 1
    P = eye(N)-(1/N)*ones(N);   %Projector Matrix
    ome_0 = 0.5;                %ome_0^-1 is Radius of circular motion
    c = zeros (2*tmax/dt,N);
    K=1;
    c(1,:)=r(1,:)-1/ome_0*sin(theta_abs(1,:));
    c(2,:)=r(2,:)+1/ome_0*cos(theta_abs(1,:));
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
        dr = [cos(theta_abs(ii,:))*v;sin(theta_abs(ii,:))*v];

        for jj = 1:N
            nu(jj) = ome_0*(1+K*P(jj,:)*c(ii:(ii+1),:).'*dr(:,jj));
        end 
        
        theta_abs(ii+1,:) = theta_abs(ii,:)+ nu * dt ;

        rx = cos(theta_abs(ii,:))*v*dt;
        ry = sin(theta_abs(ii,:))*v*dt;

        r(2*(ii+1)-1,:)= r(2*ii-1,:)+rx;
        r(2*(ii+1),:)= r(2*ii,:)+ry;
        
        c(2*(ii+1)-1,:) = r(2*(ii+1)-1,:) - 1/ome_0*sin(theta_abs(ii+1,:));
        c(2*(ii+1),:) = r(2*(ii+1),:) + 1/ome_0*cos(theta_abs(ii+1,:));
        
        plot(r(2*ii-1,:),r(2*ii,:),'b.');

        %pause(dt);
    end

    hold off;
    
end

