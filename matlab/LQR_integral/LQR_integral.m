%% Parameters
clear;
Path_ref;
v = 1.5; % m/s
L = 0.65; % m
dT = 0.1; % 100ms
Q = [150 0 0 0;0 150 0 0;0 0 10 0;0 0 0 0.001];
R = 0.05;
%% Function main
% init state
x0 = [posRef(1,1) posRef(1,2) yawRef(1)]';
u(1) = 0;
x(:,1) = x0;
t(1) = 0;
e(:,1) = [0 0 0]';
Ki = [0.1 0.1 0.01];
time_ex(1) = 0;
for i = 1:1000
    t(i+1) = i*dT;
    clk1 = clock;
    clk1 = clk1(6)*1000; %ms
    [index, ~] = find_nearest_point(x(1,i),x(2,i),length(yawRef), posRef);
    [Ae,Be,Ce] = get_linearized_model([posRef(index,1) posRef(index,2) yawRef(index)]',v,L,dT);
    Xe = [x(1,i)-posRef(index,1) x(2,i)-posRef(index,2) x(3,i)-yawRef(index) u(i)]';
    e(:,i+1) = e(:,i) + dT*[posRef(index,1)-x(1,i) posRef(index,2)-x(2,i) yawRef(index)-x(3,i)]';
    Klqr = LQR_solver(Ae,Be,Ce,Q,R);
    delta_U = -Klqr*Xe + Ki*e(:,i+1);
    % Constraint of steering angle
    % -pi/6 < u < pi/6
    if u(i) + delta_U <= -pi/6
        delta_U = -pi/6 - u(i);
    end
    if u(i) + delta_U >= pi/6
        delta_U = pi/6 - u(i);
    end
    u(i+1) = u(i) + delta_U;
    clk2 = clock;
    clk2 = clk2(6)*1000; %ms
    time_ex(i+1) = clk2 - clk1;
    x(:,i+1) = get_update(x(:,i),u(i+1),v,L,dT);
    if index == length(yawRef)
        break;
    end
    scatter(x(1,i), x(2,i), 'filled', 'red');
    hold on;
    plot(x(1,:),x(2,:));
    hold on;
    plot(posRef(index,1),posRef(index,2),'o');
    hold on;
    plot(posRef(:,1),posRef(:,2));
    hold off;
    pause(0.1);
end
subplot(3,1,1)
plot(x(1,:),x(2,:));
hold on;
plot(posRef(:,1),posRef(:,2));
hold off;
legend('actual', 'reference');
subplot(3,1,2)
plot(t,u)
hold on;
plot(t,x(3,:));
legend('steering angle', 'heading angle');
hold off;
subplot(3,1,3)
plot(t,time_ex);
legend('time execution per iteration (ms)');
hold off;
%% Linearized kinematic model at Xr(ki) = X(ki)
function [Ae,Be,Ce] = get_linearized_model(cur_X, v, L, dT)
Xr = cur_X;
Ae = [1 0 -v*sin(Xr(3))*dT 0;0 1 v*cos(Xr(3))*dT 0;0 0 1 v/L*dT;0 0 0 1];
Be = [0 0 v/L*dT 1]';
Ce = [1 0 0 0; 0 1 0 0; 0 0 1 0];
end


%% real_model
% X = [x, y, theta], u = delta
function X_next = get_update(X, delta, v, L, dT)
X_next(1) = X(1) + v*cos(X(3))*dT;
X_next(2) = X(2) + v*sin(X(3))*dT;
X_next(3) = X(3) + v/L*tan(delta)*dT;
end

%% LQR solver
function Klqr = LQR_solver(A,B,C,Q,R)
P = Q;
max_iter = 150;
eps = 0.01;
Pn = P + 1;
while(max(abs(Pn - P)) > eps)
%for i = 1:max_iter
    Pn = A'*P*A - A'*P*B/(R + B'*P*B)*B'*P*A + Q;
%     if(max(abs(Pn - P)) < eps)
%         break
%     end
    P = Pn;
end
Klqr = (B'*Pn*B+R)\(B'*Pn*A);
end

%% find nearest point
function [index,dmin] = find_nearest_point(x,y,max_index, posRef)
for i = 1:max_index
    d(i) = sqrt((x-posRef(i,1))^2 + (y-posRef(i,2))^2);
end
[dmin, index] = min(d);
end


