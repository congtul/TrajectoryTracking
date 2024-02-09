%% Parameters
clear;
Path_ref;
v = 2.2; % m/s
L = 0.65; % m
dT = 0.1; % 100ms
Np = 7;
Nc = 7;
Q = [50 0 0;0 50 0;0 0 50];
Qp = [50 0 0;0 50 0;0 0 300];
Qy = [];
M = zeros(2*Nc,Nc);
for i = 1:Nc
    M(i,1:i) = 1;
    M(i+Nc,1:i)=-1;
end
for i=1:Np-1
    Qy = blkdiag(Qy,Q);
end
Qy = blkdiag(Qy,Qp);
R = 0.05*eye(Nc);
%% Function main
% init state
% x0 = [posRef(1,1) posRef(1,2) yawRef(1)]';
x0 = [posRef(1,1) posRef(1,2) yawRef(1)]';
u(1) = 0;
x(:,1) = x0;
t(1) = 0;
time_ex(1) = 0;
x_ref(1) = x0(1);
y_ref(1) = x0(2);
yaw_ref(1) = x0(3);
for i = 1:1000
    t(i+1) = i*dT;
    clk1 = clock;
    clk1 = clk1(6)*1000; %ms
    [Ae,Be,Ce] = get_linearized_model(x(:,i), v, L, dT);
    [index, Rs, plot_ref] = create_trajectory(x(:,i),posRef,yawRef, Np, v, dT);
    x_ref(i+1) = posRef(index,1);
    y_ref(i+1) = posRef(index,2);
    yaw_ref(i+1) = yawRef(index);
    Xe = [0 0 0 u(i)]';
    N = ones(2*Nc,1);
    N(1:Nc,1) = pi/6 - u(i);
    N(Nc+1:2*Nc,1) = pi/6 + u(i);
    solution = QP_solver(Ae,Be,Ce,Qy,R,Xe,Rs,M,N,Np,Nc);
    delta_u = solution(1,1);
    u(i+1) = u(i) + delta_u;
    clk2 = clock;
    clk2 = clk2(6)*1000; %ms
    time_ex(i+1) = clk2 - clk1;
    x(:,i+1) = get_update(x(:,i), u(i+1), v, L, dT);
    % last point of path
    if index == length(yawRef)
        break;
    end
    scatter(x(1,i), x(2,i), 'filled', 'red');
    hold on;
    plot(x(1,:),x(2,:));
    hold on;
    plot(plot_ref(:,1),plot_ref(:,2),'o');
    hold on;
    plot(posRef(:,1),posRef(:,2));
    hold off;
    pause(0.1);
end
figure(1);
subplot(3,1,1)
plot(x(1,:),x(2,:));
hold on;
plot(posRef(:,1),posRef(:,2));
hold off;
legend('actual (m)', 'reference (m)');
subplot(3,1,2)
plot(t,u)
hold on;
plot(t,x(3,:));
legend('steering angle (rad)', 'heading angle (rad)');
hold off;
subplot(3,1,3)
plot(t,time_ex);
legend('time execution per iteration (ms)');
hold off;
figure(2);
subplot(3,1,1);
plot(t,x_ref);
hold on;
plot(t,x(1,:));
legend('reference', 'actual');
title('X respone');
hold off;
subplot(3,1,2);
plot(t,y_ref);
hold on;
plot(t,x(2,:));
hold off;
legend('reference', 'actual');
title('Y respone');
subplot(3,1,3);
plot(t,yaw_ref);
hold on;
plot(t,x(3,:));
legend('reference', 'actual');
title('Heading angle respone');
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

%% QP solver
function delta_U = QP_solver(A,B,C,Qy,R,X,Rs,M,N,Np,Nc)
% Y = F*x + G*delta_U
h = C;
F(1:3,:) = C*A;
for i = 2:Np
    F(3*i-2:3*i,:) = F(3*(i-1)-2:3*(i-1),:)*A;
    h(3*i-2:3*i,:) = h(3*(i-1)-2:3*(i-1),:)*A;
end
v = h*B;
G = zeros(3*Np,Nc);
G(:,1) = v;
for i = 2:Nc
    G(:,i) = [zeros(3*(i-1),1);v(1:3*(Np-i+1),1)];
end
% Jk = delta_U'*H*delta_U + 2*f*delta_U
% M*delta_U <= N
H = G'*Qy*G + R;
if max(abs(H - H')) > 0
    H = 1/2*(H + H');
end
f = G'*Qy*(F*X-Rs);
[n1,~] = size(M);
% check if violate constraints or not
delta_U = -H\f;
kk = 0;
for i = 1:n1
    if(M(i,:)*delta_U > N(i))
       kk = kk+1;
    else
       kk = kk+0;
    end
end
if(kk==0)
    return;
end
%------------ Hildreth------------------%
% Find solution for violated constraints
% Compute P, d
P = M*(H\M');
d = (N + M*(H\f));
[n,m] = size(d);
% init lamda = 0
x_ini=zeros(n,m);
lambda=x_ini;
al = 1;
kmax = 100000;
while(al>10e-8)
    lambda_p = lambda;
    for i=1:n
        w= P(i,:)*lambda-P(i,i)*lambda(i,1);
        w=w+d(i,1);
        la=-w/P(i,i);
        lambda(i,1)=max(0,la);
    end
al=(lambda-lambda_p)'*(lambda-lambda_p);
% if (al<10e-8) 
%     break;
% end
end
delta_U = -H\f - H\M'*lambda;
% %-----------Active-set-----------------%
% delta_U = quadprog(H,f,M,N);
% if max(abs(H - H')) > 0
%     H = 1/2*(H + H');
% end
% %Init Delta_U and working set W
% delta_U = zeros(Nc,1);
% M_w = M(1,:)
% N_w = N(1,:)
% % delta_U(1,1) = N(1,1)/M(1,1)
% z = 0;
% for z = 1:20
% % find direction d through J = 1/2d*H*d + g*d, M_w*d = 0, g = (H*delta_U+f)
% g = H*delta_U + f;
% lamda_d = -((M_w/H*M_w'))\(M_w/H*g);
% d = -H\(M_w'*lamda_d+g)
% if max(abs(d)) < 10e-6
%     % Check if working set is optimal
%     lamda = -((M_w/H*M_w'))\(M_w/H*f+N_w);
%     [lamda_q, index] = min(lamda);
%     if lamda_q >= 0
%         delta_U
%         break; % Working set is optimal
%     else
%         % delete 1 constraints in working set
%         M_w(index,:) = [];
%         N_w(index,:) = [];
%     end
% else
%     % Calculate alpha for update iteration
%     temp = M*d;
%     min_dist = 99999;
%     index_W = 0;
%     [n,~] = size(M);
%     for i=1:n
%         if temp(i,1) > 0
%             dist = (N(i,1) - M(i,:)*delta_U)/temp(i,1);
%             if dist < min_dist
%                 min_dist = dist;
%                 index_W = i;
%             end
%         end
%     end
%     alpha = min(min_dist,1);
%     delta_U = delta_U + alpha*d;
%     if min_dist < 1
%         [n,~] = size(M_w);
%         M_w(n+1,:) = M(index_W,:);
%         N_w(n+1,:) = N(index_W,:);
%     end
% end
% end
end

%% find nearest point
function [index,dmin] = find_nearest_point(x,y,max_index, posRef)
for i = 1:max_index
    d(i) = sqrt((x-posRef(i,1))^2 + (y-posRef(i,2))^2);
end
[dmin, index] = min(d);
end

%% Create MPC trajectory
function [index,Rs, plot_ref] = create_trajectory(X, posRef, yawRef, Np, v, dT)
[index,~] = find_nearest_point(X(1),X(2),length(yawRef),posRef);
mpc_start = index;
mpc_end = mpc_start;
distance = v*Np*dT;
Rs = zeros(3*Np,1);
dist_ref = 0;
while (dist_ref < distance) && (mpc_end < length(yawRef))
    mpc_end = mpc_end + 1;
    dist_ref = dist_ref + sqrt((posRef(mpc_end,1)-posRef(mpc_end-1,1))^2+(posRef(mpc_end,2)-posRef(mpc_end-1,2))^2);
end
for i = 1:Np
    index_ref = mpc_start + round((mpc_end - mpc_start)/Np*i);
    Rs(3*(i-1)+1,1) = posRef(index_ref,1) - (X(1) + i*v*cos(X(3))*dT);
    Rs(3*(i-1)+2,1) = posRef(index_ref,2) - (X(2) + i*v*sin(X(3))*dT);
    Rs(3*(i-1)+3,1) = yawRef(index_ref) - X(3);
    plot_ref(i,1) = posRef(index_ref,1);
    plot_ref(i,2) = posRef(index_ref,2);
%     Rs(3*(i-1)+1,1) = posRef(index_ref,1) - (X(1));
%     Rs(3*(i-1)+2,1) = posRef(index_ref,2) - (X(2));
%     Rs(3*(i-1)+3,1) = yawRef(index_ref) - X(3);
end
end




