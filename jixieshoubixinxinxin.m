% s = 'Rz(q1).Rx(q2).Ty(L1).Rx(q3).Ty(L2)';
% dh = DHFactor(s);
% dh.tool;
% L1 = 0.2;
% L2 = 0.2;
% L3 = 0.1;
% leg = eval(dh.command('leg'))
% % transl(leg.fkine([0,0,0]))'
% leg.plot([0,0,0])
% set(gca,'Zdir','reverse');
% view(137,48)
% transl(leg.fkine([0.2,0,0]))
% transl(leg.fkine([0,0,0.2]))


% dh.display
% cmd = dh.command('puma');
% robot = eval(cmd)

L1=Link([0 0 0 pi/2      ]); %定义连杆的D-H参数
L2=Link([0 0 0.1 0       ]);
L3=Link([0 0 0.1 0       ]);
leg=SerialLink([L1 L2 L3],'base',transl(0,0,-0.1)); 
% leg.plot([0,0,0],'nobase','noshadow')
xf = 50; 
xb = -xf;
y = 50;
zu = 50;
zd = 50;
% path = [xf y zd;xb y zd;xb y zu;xf y zu;xf y zd] * 1e-3;
path =[0.05 0.05 0.02;0.05 0.05 0.08;-0.05 0.05 0.02;-0.05 0.05 0.08;0.05 0.05 0.02]
p = mstraj(path,[],[0,3,0.25,0.5,0.25]',path(1,:),0.1,0);


% Tp = transl(p);
%  Tp = homtrans(transl(0,0,0),Tp);
% a=squeeze(Tp(1,4,:));
% b=squeeze(Tp(2,4,:));
% c=squeeze(Tp(3,4,:));
%  plot3(a,b,c)
% p = mstraj(path,[3 3 3],[],[],0.2,3);
qcycle = leg.ikine(transl(p),'mask',[1 1 1 0 0 0]);
plot3(qcycle(:,1),qcycle(:,2),qcycle(:,3))
% leg.plot(p)
leg.plot(qcycle,'loop','nobase','noshadow')
