%% Parameters
clear;
Path_ref;
v = 2.2; % m/s
L = 0.65; % m
dT = 0.1; % 100ms
%Q = [150 0 0 0;0 150 0 0;0 0 10 0;0 0 0 0.001];
Q = zeros(6,6);
Q(1,1) = 150;
Q(2,2) = 150;
Q(3,3) = 50;
Q(4,4) = 0.01;
Q(5,5) = 0.01;
Q(6,6) = 0.01;
R = 0.05;
Np = 10;
p = 0.1;
%% Function main
% init state
x0 = [posRef(1,1) posRef(1,2) yawRef(1)]';
u(1) = 0;
x(:,1) = x0;
t(1) = 0;
e(:,1) = [0 0 0]';
% Ki = [0.1 0.1 0.01];
time_ex(1) = 0;
for i = 1:1000
    t(i+1) = i*dT;
    clk1 = clock;
    clk1 = clk1(6)*1000; %ms
    [index, ~] = find_nearest_point(x(1,i),x(2,i),length(yawRef), posRef);
    [Ae,Be] = get_linearized_model([posRef(index,1) posRef(index,2) yawRef(index)]',v,L,dT);
    Xe = [x(1,i)-posRef(index,1) x(2,i)-posRef(index,2) x(3,i)-yawRef(index)]';
    % e(:,i+1) = e(:,i) + dT*[posRef(index,1)-x(1,i) posRef(index,2)-x(2,i) yawRef(index)-x(3,i)]';
    e(:,i+1) = e(:,i) - dT*Xe;
    Klqr = LQR_solver(Ae,Be,Q,R);
    %Klqr = dlqr(Ae,Be,Q,R)
    Kx = Klqr(:,1:3);
    Ki = Klqr(:,4:6);
    u_lqr = -Kx*Xe + Ki*e(:,i+1);
    [u_pure_pursuit,point_ahead] = Pure_pursuit_controller(x(:,i),posRef,yawRef,Np,v,dT,L);
    u(i+1) = p*u_lqr + (1-p)*u_pure_pursuit;
    % Constraint of steering angle
    % -pi/6 < u < pi/6
    if u(i+1) <= -pi/6
        u(i+1) = -pi/6;
    end
    if u(i+1) >= pi/6
        u(i+1) = pi/6;
    end
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
    plot(posRef(point_ahead,1),posRef(point_ahead,2),'o');
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
function [Ae,Be] = get_linearized_model(cur_X, v, L, dT)
Xr = cur_X;
A = [0 0 -v*sin(Xr(3));
     0 0  v*cos(Xr(3));
     0 0            0];
B = [0 0 v/L]';
C = eye(3);
Ae = [A zeros(3,3); -C zeros(3,3)];
Be = [B; [0 0 0]'];
Ae = Ae*dT + eye(6);
Be = Be*dT;
end


%% real_model
% X = [x, y, theta], u = delta
function X_next = get_update(X, delta, v, L, dT)
X_next(1) = X(1) + v*cos(X(3))*dT;
X_next(2) = X(2) + v*sin(X(3))*dT;
X_next(3) = X(3) + v/L*tan(delta)*dT;
end

%% LQR solver
function Klqr = LQR_solver(A,B,Q,R)
P = Q;
max_iter = 150;
eps = 100;
Pn = P + 1;
for i = 1:max_iter
    Pn = A'*P*A - A'*P*B/(R + B'*P*B)*B'*P*A + Q;
    if(max(max(abs(Pn - P))) < eps)
         break
    end
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
%% Pure pursuit controller
function [u,point_ahead] = Pure_pursuit_controller(X, posRef, yawRef, N, v,dT, L)
[index,~] = find_nearest_point(X(1),X(2),length(yawRef),posRef);
point_ahead = index;
distance = v*N*dT;
dist_ref = 0;
while (dist_ref < distance) && (point_ahead < length(yawRef))
    point_ahead = point_ahead + 1;
    dist_ref = dist_ref + sqrt((posRef(point_ahead,1)-posRef(point_ahead-1,1))^2+(posRef(point_ahead,2)-posRef(point_ahead-1,2))^2);
end
alpha = atan2(posRef(point_ahead,2)-X(2),posRef(point_ahead,1)-X(1)) - X(3);
ld = sqrt((posRef(point_ahead,1)-X(1))^2 + (posRef(point_ahead,2)-X(2))^2);
u = atan2(2*L*sin(alpha),ld);
end


