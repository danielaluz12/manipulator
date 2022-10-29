%pose_1 joint states with time
pose1_Time= pose1_Time*10^9
figure
plot(pose1_Time,pose1_position_0,pose1_Time,pose1_position_1,pose1_Time,pose1_position_2,pose1_Time,pose1_position_3,pose1_Time,pose1_position_4)
legend({'Joint_0','Joint_1','Joint_2','Joint_3','Joint_5'},'location','northwest')
title('Pose1')

%pose_2 joint states with time
figure
plot(pose2_Time,pose2_position_0,pose2_Time,pose2_position_1,pose2_Time,pose2_position_2,pose2_Time,pose2_position_3,pose2_Time,pose2_position_4)
legend({'Joint_0','Joint_1','Joint_2','Joint_3','Joint_5'},'location','northwest')
title('Pose2')

%pose_3 joint states with time
figure
plot(pose3_Time,pose3_position_0,pose3_Time,pose3_position_1,pose3_Time,pose3_position_2,pose3_Time,pose3_position_3,pose3_Time,pose3_position_4)
legend({'Joint_0','Joint_1','Joint_2','Joint_3','Joint_5'},'location','northwest')
title('Pose3')

%plotting 3D poses in end-effector coordinates
x1 = ee_pose1_posepositionx;
y1 = ee_pose1_posepositiony ;
z1 = ee_pose1_posepositionz;

x2 = ee_pose2_posepositionx;
y2 = ee_pose2_posepositiony ;
z2 = ee_pose2_posepositionz;

x3 = ee_pose3_posepositionx;
y3 = ee_pose3_posepositiony ;
z3 = ee_pose3_posepositionz;

figure
plot3(x1,y1,z1,'o',x2,y2,z2,'o',x3,y3,z3,'o');
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
legend({'Pose_1','Pose_2','Pose_3'},'location','northwest')




%3d plots of tracjetory for poses with Peter corke- using jointa_states-
%result of IK
% calculo númericos

% |  theta |         d |         a |     alpha |  sigma|  offset |
%L(1) = Link([0  0.06    0     3.14/2     0],'qlim',[0,3.14/2]);
% L(2) = Link([0  0     0.12       0    0]);
% L(3) = Link([0  0     0.105        0   0]);
% L(4) = Link([0  0     0.05        0   0]);

L(1) = Link('revolute', 'd', 0.12, 'a',  0, 'alpha',  3.14/2,'qlim',[0,3.14]);
L(2) = Link('revolute', 'd',0.0, 'a',   0.12 , 'alpha', 0.0,'qlim',[-0.6,2.8]);
L(3) = Link('revolute', 'd', 0.0, 'a',  0.105, 'alpha',   0.0,'qlim',[-1.39,1.39]);
L(4) = Link('revolute', 'd', 0.0, 'a',  0.12 , 'alpha',   0.0,'qlim',[-3.14,3.14]);

% Criação do robô
N = SerialLink(L, 'name', 'A')
q1 =[0,0,0,0]
T1 = N.fkine(q1);   %cinametica direta
figure
N.plot(q1)
N.teach

q1 = N.getpos()

%pose1
q1_1 = [3.0772    0.3400   -0.2224   -1.6956]      
q2_1= [3.0772    2.7880    0.9730   -0.8164] 

%pose2
q1_2 = [3.0772    0.7628   -0.5282   -1.8212]      
q2_2= [1.6014    0.8988   -0.5282   -0.4396]

%pose3
q1_3 = [0    0.5900   -0.7228   -1.4444]      
q2_3= [0    1.4428    0.8618    0.3140]


Tempo = 0:0.2:5;

%dynamica direta
Traj = jtraj(q1_1,q2_1,Tempo)
%dynamica direta valores
% [Q QD QDD]=jtraj(q1,q2,Tempo)

Mov =N.fkine(Traj)
X=transl(Mov)
%plot da trajetoria:
plot3(X(:,1),X(:,2),X(:,3))

%trajetoria + robo
N.plot(q1_1)
hold
% plot3(X(:,1),X(:,2),X(:,3))
N.plot(Traj)