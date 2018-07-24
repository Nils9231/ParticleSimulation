dt = 0.01;                              % time step for animation
t = 0:dt:15;
pC = zeros(length(t),2);
pC(1,:) = [4 10];                       % initial position of A
vC = zeros(length(t),2);
vC(1,:) = [2.3100 -0.5500];             % initial velocity of A
beta = atan(vC(1,2)/vC(1,1));
alpha = 0:(2*pi/length(t)):2*pi;
delta = beta-alpha;

for i = 1:length(t)                     % t is time variable 
   vC(i+1,1)= vC(i,1)*(cos(delta(i+1))/cos(delta(i)));
   vC(i+1,2)= vC(i,2)*(sin(delta(i+1))/sin(delta(i)));
end

for i = 2:length(t)                     % t is time variable 
    pC(i,:) = pC(i-1,:) + vC(i-1,:)*dt;  % update position
end

clf;                                    % Clear the figure 
axis square;
axis([-20 20 -20 20]);
grid on;
hold on;                                % Keep plot from erasing
title('Motion animation');
xlabel('x (m)');  
ylabel('y (m)');
for i = 1:length (t)
plot(pC(i,1),pC(i,2),'b.');
%pause(dt);
end
hold off;
