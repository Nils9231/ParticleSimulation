%% 
%   Particle Dynamics and State Feedback Contro from Napora and Paley:
%   "Observer-Based Feedback Control For Stabilization od Collective
%   Motion"
%
%   first order steering
close all
%% Deklarations/Definitions
% Timestepsize
dt = 0.01;
% Simulation Time (s)
tmax = 30;
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
drx = zeros(1,N);
dry = zeros(1,N);
% Time-Derivation of Speed Orientation 
nu = zeros (1,N);
% Control Gains 
K = -1; % Swarm Gain
Kt = 3;   % Tangetial Gain
%   K<0 for straight-line Motion (parrallel trajectories)
%   K>0 for balanced Motion (Sum of all velocities equal zero)
% if straight Line Motion Tangent_target:
r_Tx=-20:0.01:20;
r_Ty= 1* r_Tx;
r_T=[1;1];

theta_T = atan2(r_T(2),r_T(1));
% Punkte auf Tangente, mit minimalem Abstand zum  Boot
R_T = zeros(2,1);
% Vektor zur Tangente
r_target = zeros(2,1);
% Circular Motion?
circular_motion = 0; % 1 Yes 0 No



%% timesteps

if circular_motion == 0
if K<0
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
%         for jj = 1:N
%             for kk = 1:N
%                 nu(jj) = nu(jj) + sin(theta_abs(ii,kk)-theta_abs(ii,jj));
%             end
%         end 
%         nu = - (K/N)*nu;
        for jj = 1:N
            R_T = [norm([r(2*ii-1,jj);r(2*ii,jj)])*sin(theta_T)*cos(theta_T);norm([r(2*ii-1,jj);r(2*ii,jj)])*sin(theta_T)*sin(theta_T)]+r_T*Kt;
            r_target = R_T-[r(2*ii-1,jj);r(2*ii,jj)];
            %theta_target = acos(r_target(1)/norm(r_target));
            theta_target = atan2(r_target(2),r_target(1));
            nu(jj) = sin(theta_target-theta_abs(ii,jj));
        end
        nu = - (K)*nu;
        
        %Winkel zur Tangente berechnen
        
        
        theta_abs(ii+1,:) = theta_abs(ii,:)+ nu * dt ;
        
        drx = cos(theta_abs(ii,:))*v*dt;
        dry = sin(theta_abs(ii,:))*v*dt;

        r(2*(ii+1)-1,:)= r(2*ii-1,:)+drx;
        r(2*(ii+1),:)= r(2*ii,:)+dry;

        plot(r(2*ii-1,:),r(2*ii,:),'b.');

        pause(dt);
    end
elseif K>0
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

        drx = cos(theta_abs(ii,:))*v*dt;
        dry = sin(theta_abs(ii,:))*v*dt;

        r(2*(ii+1)-1,:)= r(2*ii-1,:)+drx;
        r(2*(ii+1),:)= r(2*ii,:)+dry;

        plot(r(2*ii-1,:),r(2*ii,:),'b.');

        pause(dt);
    end
end

    hold off;
elseif circular_motion == 1
    P = eye(N)-(1/N)*ones(N);   %Projector Matrix
    ome_0 = 0.2;                %ome_0^-1 is Radius of circular motion
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
        dr = [cos(theta_abs(ii,:));sin(theta_abs(ii,:))];

        for jj = 1:N
            nu(jj) = ome_0*(1+K*P(jj,:)*c(2*ii-1:2*ii,:).'*dr(:,jj));
        end 
        
        theta_abs(ii+1,:) = theta_abs(ii,:)+ nu * dt ;

        drx = cos(theta_abs(ii,:))*v*dt;
        dry = sin(theta_abs(ii,:))*v*dt;

        r(2*(ii+1)-1,:)= r(2*ii-1,:)+drx;
        r(2*(ii+1),:)= r(2*ii,:)+dry;
        
        c(2*(ii+1)-1,:) = r(2*(ii+1)-1,:) - 1/ome_0*sin(theta_abs(ii+1,:));
        c(2*(ii+1),:) = r(2*(ii+1),:) + 1/ome_0*cos(theta_abs(ii+1,:));
        
        plot(r(2*ii-1,:),r(2*ii,:),'b.');

        pause(dt/2);
    end

    hold off;
    
end

