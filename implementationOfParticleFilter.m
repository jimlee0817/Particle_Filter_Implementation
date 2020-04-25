% PME 525600 
% Special Topics in Mobile Robots and Self-Driving Cars 
% HW#3
% Author: Yun-Jin, Li
% -----------------------------------------------------

% Implementation of the particle filter on a 
% previous motion model in Hw#2

clc
clear all
close all

N = 10000; % number of particles
Xt = [0; 0]; % initial condition, given [positionX, velocityX]' 
dT = 1;
A = [1 dT; ...
    0 1];
B = [0.5 * dT^2; ...
    dT];
t =  0;
posteriorXt_1 = zeros(2, N);

    
while t < 4
    priorXt = zeros(2, N);
    posteriorXt = zeros(2, N);
    w = zeros(1, N);
    for i = 1:N
        u = normrnd(0, 1, [1, N]); % create a normal distribution random number where mu = 0, and covariance = 1
        priorXt(:, i) = A * posteriorXt_1(:, i) + B * u(1, i);
        w(:, i) = N^-1;
    end
    
    % resample
    r = 0 + rand * (N^-1 - 0);
    c = w(1, 1);
    idx = 1;
    for m = 1:N
        U = r + (m - 1) * N^-1;
        while U > c
            idx = idx + 1;
            c = c + w(1, idx);
        end
        posteriorXt(:, m) = priorXt(:, idx);
    end
    
    figure(t + 1);
    plot(posteriorXt(1,:), posteriorXt(2,:), '.');
    axis([-20 20 -20 20]);
    title(['t = ', num2str(t + 1)]);
    xlabel('${x_t}(m)$','interpreter','latex');
    ylabel('$\dot{x_t}(m)$','interpreter','latex');
    posteriorXt_1 = posteriorXt;
    t = t + 1;
end

priorXt = zeros(2, N);
posteriorXt = zeros(2, N);
w = zeros(1, N);
for i = 1:N
    u = normrnd(0, 1, [1, N]); % create a normal distribution random number where mu = 0, and covariance = 1
    priorXt(:, i) = A * posteriorXt_1(:, i) + B * u(1, i);
    w(:, i) = gaussmf(5,[sqrt(10) priorXt(1, i)]);
end


% resample
r = 0 + rand * (N^-1 - 0);
c = w(1, 1)/sum(w);
idx = 1;
for m = 1:N
    U = r + (m - 1) * N^-1;
    while U > c
        idx = idx + 1;
        c = c + w(1, idx)/sum(w);
    end
    posteriorXt(:, m) = priorXt(:, idx);
end

figure(5)
plot(posteriorXt(1,:), posteriorXt(2,:), '.');
axis([-20 20 -20 20]);
title('t = 5');
xlabel('${x_t}(m)$','interpreter','latex');
ylabel('$\dot{x_t}(m)$','interpreter','latex');

