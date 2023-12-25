%% THE NUMERICAL VALUES OF THE PARAMETERS USED IN THE SIMULATION:
% Parameters for the dynamic model of the manipulators
m1=1.2; m2=1; %masses of the links.
l1=1.5; l2=1.5; %lengths of links.
lc1=0.75; lc2=0.75; %the distances from the origins of the link frames to the centers of masses.
g=9.81; %the gravitational acceleration.
I1=0.225; I2=0.1875; %the link moments of inertia.
J1=0; J2=0; %the motor moments inertia.
F1=0; F2=0; %the friction constants.
p1=m1*lc1^2+m2*l1^2+2*m2*l1*lc2^2+I1;
p2=m2*lc2^2+I2;
p3=m2*l1*lc2;
p4=0;
p5=0;
p6=(m1*lc1+m2*l1)*g;
p7=m2*lc2*g;
p8=m2*lc2;
p9=0;
p10=0;

% Disturbances:
% dM=[sin(t);cos(t)];
% dE=[0.5*sin(pi/2*t);0.5*sin(pi/2*t)];
% x=h(q1,q2)+xi0=[l1*cos(q1)+l2*cos(q1+q2);l1*sin(q1)+l2*sin(q1+q2)]+x0;
% J(1)=[-l1*sin(q1)-l2*sin(q1+q2),-l2*sin(q1+q2);
%       +l1*cos(q1)+l2*cos(q1+q2),+l2*cos(q1+q2)];

% Desired formation is a square with side length of 0.4 m.
% Connection between the agents:
G1 = [1 2 3 4 1];
G2 = [2 3 4 1 3];
G = graph(G2,G1);
%figure();
%plot(G)
L=full(laplacian(G));
B=full(incidence(G));
B=[1 0 0 -1 1;-1 1 0 0 0;0 -1 1 0 -1;0 0 -1 1 0];

% Positions of the bases of the 4 manipulators
x1_0=[0;0]; x2_0=[5;0]; x3_0=[5;3]; x4_0=[0;3];

% Initial joint positions:
q1_0=[-0.1;pi/3+0.2]; q2_0=[pi/2;2*pi/3]; q3_0=[pi-0.2;pi/3+0.2]; q4_0=[0.2;-pi/3-0.2];

% Initial joint velocities:
dq1_0=[0;0]; dq2_0=[0;0]; dq3_0=[0;0]; dq4_0=[0;0];

% Constroller parameters:
Kp=800; Kd=600;

% Matrices for compensators:
AM=blkdiag([0 1;-1 0],[0 1;-1 0]);
AE=blkdiag([0 pi/2;-pi/2 0],[0 pi/2;-pi/2 0]);
GammaM=blkdiag([1 0],[1 0]);
GammaE=blkdiag([1 0],[1 0]);
% Parameter to turn on of turn off the disturbance.
ON_OFF=0;
%parameters for the extended observer:
A2=[10 1;1 10];
A1=[30 3;3 30];
A0=[30 3;3 30];
kapa=100;
M0=[p1+p2+p4,p2;p2,p2+p5]+[2*p3,p3;p3,0]*cos(0)*0.5;
N=100;
%% After running the simulation, one can evalute this section:
% Plot end effectors trajectories in the space.
x1=out.x1(:,2:1:3);
x2=out.x2(:,2:1:3);
x3=out.x3(:,2:1:3);
x4=out.x4(:,2:1:3);
number=length(x1(:,1));
x=[x1(:,1),x2(:,1),x3(:,1),x4(:,1)];
y=[x1(:,2),x2(:,2),x3(:,2),x4(:,2)];
XY0=[[x1(1,1) x2(1,1) x3(1,1) x4(1,1)]',[x1(1,2) x2(1,2) x3(1,2) x4(1,2)]'];
XYlast=[[x1(number,1) x2(number,1) x3(number,1) x4(number,1)]',[x1(number,2) x2(number,2) x3(number,2) x4(number,2)]'];

figure();
hold on;
plot(x,y,'LineWidth',2);
s1=scatter(XY0(:,1),XY0(:,2),700,'b');
s1.Marker='square';
s1.LineWidth=2;
s1.MarkerFaceColor='w';
s1=scatter(XYlast(:,1),XYlast(:,2),700,'r');
s1.Marker='square';
s1.LineWidth=2;
s1.MarkerFaceColor='w';
i=number;
for k=1:1:length(G1)
    plot([x(i,G1(k)) x(i,G2(k))],[y(i,G1(k)) y(i,G2(k))],'r','LineWidth',1);
end

for k=1:1:length(G1)
    plot([x(1,G1(k)) x(1,G2(k))],[y(1,G1(k)) y(1,G2(k))],'--k','LineWidth',1);
end

s1=scatter(XY0(:,1),XY0(:,2),650,'b');
s1.Marker='square';
s1.LineWidth=2;
s1.MarkerFaceColor='w';
s1=scatter(XYlast(:,1),XYlast(:,2),650,'r');
s1.Marker='square';
s1.LineWidth=2;
s1.MarkerFaceColor='w';
w = strsplit(sprintf('%d\n', [1:1:4], '\n'));
text(XY0(:,1), XY0(:,2), w(1:1:4)', 'HorizontalAlignment','center', 'VerticalAlignment','middle','Fontsize',20)
text(XYlast(:,1),XYlast(:,2), w(1:1:4), 'HorizontalAlignment','center', 'VerticalAlignment','middle','Fontsize',20)
leg=legend('Trajectory of agent 1','Trajectory of agent 2','Trajectory of agent 3','Trajectory of agent 4','Initial positions','Final positions','FontSize',17);
%pbaspect([1 1 1])
sqrt(sum((XYlast(3,:)-XYlast(1,:)).^2))-0.4*sqrt(2)
sqrt(sum((XYlast(2,:)-XYlast(1,:)).^2))-0.4
sqrt(sum((XYlast(2,:)-XYlast(3,:)).^2))-0.4
sqrt(sum((XYlast(4,:)-XYlast(3,:)).^2))-0.4
sqrt(sum((XYlast(1,:)-XYlast(4,:)).^2))-0.4
%% Plot the planners while they moving:
q1=out.q1(:,2:1:3);
q2=out.q2(:,2:1:3);
q3=out.q3(:,2:1:3);
q4=out.q4(:,2:1:3);
L1 = Link('revolute','d', 0, 'a', l1, 'alpha', 0);
L2 = Link('revolute','d', 0, 'a', l2, 'alpha', 0);
Robot1=   SerialLink([L1 L2]); Robot1.name='Robot1';
Robot2 = SerialLink(Robot1, 'name', 'Robot2');
Robot3 = SerialLink(Robot1, 'name', 'Robot3');
Robot4 = SerialLink(Robot1, 'name', 'Robot4');
Robot1.base.t=[x1_0;0];
Robot2.base.t=[x2_0;0];
Robot3.base.t=[x3_0;0];
Robot4.base.t=[x4_0;0];

for i=1:100:number
Robot1.plot(q1(i,:),'workspace',[-0.5 5 -1 3 -5 5]);
hold on;
Robot2.plot(q2(i,:),'workspace',[-0.5 5 -1 3 -5 5]);
Robot3.plot(q3(i,:),'workspace',[-0.5 5 -1 3 -5 5]);
Robot4.plot(q4(i,:),'workspace',[-0.5 5 -1 3 -5 5]);
end