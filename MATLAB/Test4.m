%% Clearing the woekspace
tic % to start the stopwatch
clear all
clc

%% Inital Data
x0 = [0;0;pi/3;10;-2];   % x; y; theta; V0; speed of pedestrian
u0 = [0; 0];         % throttle delta; steering input

T = 0.5;    % Horizon
t = 0.05;   % time of each step
N = T/t;    % No. of steps
CL = 5;     % Car Length

%% MPC
import casadi.*
x = MX.sym('x',5);  % variables
u = MX.sym('u',2);  % inputs

theta = x(3);
V = x(4);
delta = u(2);
% xdottrue = [cos(x(3))*x(4);  sin(x(3))*x(4);  x(4)*tan(u(1))/CL;   0.5*u(2); 2;];
% ft = Function('ft',{x,u},{xdottrue},{'x','u'},{'xdottrue'});

% update equation
xdot = [-V*sin(theta)*theta+cos(theta)*V;
        V*cos(theta)*theta+sin(theta)*V;
        V*tan(delta)/CL+delta*(V*(tan(delta)^2+1)/CL);
        0.5*u(1);
        0.5;];

f = Function('f',{x,u},{xdot},{'x','u'},{'xdot'});  % creating a function of update equation
dae = struct('x',x,'p',u,'ode',f(x,u));             % defining a differential algebraic equation

intg_options = struct('tf',t,'simplify',true);      % defining the integrator

intg = integrator('intg','rk',dae,intg_options);    % integrating the dae for t secs
% disp(F)

res = intg('x0',x,'p',u);
xnext = res.xf;     % storing results of integration as the input for next step

F = Function('F',{x,u},{xnext},{'x','u'},{'x_next'});   % creating a function of the above steps
% F([0;0;0;20],[1;1])

opti = casadi.Opti();   % creating a casadi Opti Stack

x = opti.variable(5,N+1); % Decision variables for state trajetcory
u = opti.variable(2,N);   % Inputs
p = opti.parameter(5,1);  % Parameter (not optimized over)

% optimization function
opti.minimize(0.5*sumsqr(x(1,:))+0.5*sumsqr(x(3,:))-50*log(sumsqr(x(1:2,1:N/2)-[x(5,1);50]))-50*log(sumsqr(x(1:2,1:N/2)-[x(5,1);20]))+sumsqr(u(:,1:N-1)-u(:,2:N))^2);

% constraints

% the variables at the next step should be equal to the output from the update equation function
for k=1:N
  opti.subject_to(x(:,k+1)==F(x(:,k),u(:,k))); 
end

% throttle should be between -1 and 1
opti.subject_to(-1<=u(1,:)<=1);

% steering angle should be between -0.5 and 0.5 rads
opti.subject_to(-0.5<=u(2,:)<=0.5);

% 1 st state should be equal to initial state which is fed later
opti.subject_to(x(:,1)==p);

% Choosing a solver
p_opts = struct('expand',true);
s_opts = struct('print_level',0);
opti.solver('ipopt',p_opts,s_opts);

Time = 10; % Time of simulation
xnew = DM(5,Time/t+1); % matrix to store new states
xnew(:,1) = x0; % defining initial state

% For Visualization
fig = figure(3);
clf(fig)
% fig.WindowState='maximized';
hold on
xlim([-25 25])
ylim([0 50])
curve = animatedline('LineWidth',2);
h(1) = plot(0,0);
poi1(1) = plot(0,0);
poi2(1) = plot(0,0);
k(1) = plot(0,0);

for i=1:Time/t
% And choose a concrete value for p
% opti.minimize(sumsqr(x(1,:))+500/sumsqr(x(1:2,1:N/2)-[0;50])+sumsqr(u(:,1:N-1)-u(:,2:N))^2);

opti.set_value(p,xnew(:,i)); % Setting p to the previous state value at each time step
sol = opti.solve(); %Solving the problem
xnew(:,i+1) = xnew(:,i) + t.*f(sol.value(x(:,1)),sol.value(u(:,1))); % Calculating the value at the next step
unew(:,i) = sol.value(u(:,1)); % SToring the inputs


% For Visualization
delete(poi1(i))
delete(poi2(i))
poi1(i+1) = plot(full(xnew(5,i+1)),50,'b.','MarkerSize',25,'DisplayName','Obstacle');
viscircles([full(xnew(5,i+1)),50],1.5,'LineStyle','--','LineWidth',1,'Color','b');
poi2(i+1) = plot(full(xnew(5,i+1)),20,'b.','MarkerSize',25,'DisplayName','Obstacle');
viscircles([full(xnew(5,i+1)),20],1.5,'LineStyle','--','LineWidth',1,'Color','b');
delete(h(i))
h(i+1) = plot(sol.value(x(1,1:N)),sol.value(x(2,1:N)),'DisplayName',num2str(i));
addpoints(curve,full(xnew(1,i+1)),full(xnew(2,i+1)))
drawnow
delete(k(i))
k(i+1) = patch('XData',[full(xnew(1,i+1))-0.5 full(xnew(1,i+1))+0.5 full(xnew(1,i+1))+0.5 full(xnew(1,i+1))-0.5],'YData',[full(xnew(2,i+1))-2 full(xnew(2,i+1))-2 full(xnew(2,i+1))+2 full(xnew(2,i+1))+2],'FaceColor','g');
rotate(k(i+1),[0 0 1], (-pi/3+full(xnew(3,i+1)))*180/pi,[full(xnew(1,i+1)) full(xnew(2,i+1)) 0])
% k(i+1) = rectangle('Position',[full(xnew(1,i+1))-0.1,full(xnew(2,i+1))-5,0.2,10]);
frame(i) = getframe(gcf);
buff = 20;
% axis([full(xnew(1,i+1))-buff,full(xnew(1,i+1))+buff,full(xnew(2,i+1))-buff,full(xnew(2,i+1))+buff])
axis([-buff,buff,full(xnew(2,i+1))-buff,full(xnew(2,i+1))+buff])
end
toc % to stop the stopwatch
% For creating Video
video = VideoWriter('mpc2','MPEG-4');
open(video)
writeVideo(video,frame)
close(video)

% xtrue=full(xnew);
% plot(xtrue(1,:),xtrue(2,:),'-*','LineWidth',2.5,'DisplayName','Actual Path')
% legend(gca)
%% Plost for state variables and inputs
figure(1)
tgrid = linspace(0,Time+t,Time/t+1);
% hold on
plot(tgrid,full(xnew).*[1;1;1;1;1],'LineWidth',2);
xlabel('Time (sec)');
legend('Position of car X','Position of car Y','Orintation of car \theta','Velocity of the car','Speed of pedestrians');
ylabel('Value of state variables')
title('Value of the state variable with time')
grid on
grid minor
figure(2)
stairs(tgrid, [unew [nan;nan]]', '-.','LineWidth',2);
legend('Throttle','Steering angle');
xlabel('Time (sec)')
ylabel('Value of the output variables')
title('Value of outputs')
grid on
grid minor
ylim([-1.25 1.25])
figure(3)
hold on
% plotting the results for the report
figure(4)
f1 = full(xnew).*[1;1;1;1;1];
plot(f1(1,:),f1(2,:),'k','LineWidth',2);
axis equal
xlabel('World Position X');
ylabel('World Position Y');
title('Trajectory of the car');
hold on
labels = {'label 1','label 2','label 3'};
plot(full(xnew(5,50)),20,'b.','MarkerSize',25,'DisplayName','Obstacle');
hold on
viscircles([full(xnew(5,50)),20],1.5,'LineStyle','--','LineWidth',1,'Color','b');
hold on
plot(full(xnew(5,50)),50,'b.','MarkerSize',25,'DisplayName','Obstacle');
hold on
viscircles([full(xnew(5,50)),50],1.5,'LineStyle','--','LineWidth',1,'Color','b');
legend('Path','Obstacle 1','Obstacle 2');

