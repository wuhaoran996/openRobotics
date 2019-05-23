% L(1)=Link([0 0 1 0]);
% L(2)=Link([0 0 1 0]);
% twolink = SerialLink(L,'name','aaa');
% twolink.fkine([0 0]);
% twolink.fkine([pi/4 pi/4])
% twolink.plot([0 0]);
% twolink.plot([pi/4 -pi/4])
% p560.plot(qn)
% p560.fkine(qz)
% T = p560.fkine(qn)
% qi =p560.ikine6s(T)
%  p560.plot(qi)
% 
clear all
mdl_puma560
T = p560.fkine(qn);
q1i = p560.ikine6s(T,'ru');
% path = [2 0 4 ;2 0 0;0 0 0;0 4 0;2 4 0;0 4 2;0 2 2;0 2 0;2 2 0 ;2 2 2  ];
%%%我们画一个哲学符号
%  path = [2 0 0;3 -2 0;2 -4 0;0 -5 0;-2 -4 0;-3 -2 0;...
%      -2 0 0;0 1 0;2.3 0 0;0 1 0;0 4 0;-3 2 0;0 4 0;3 2 0];
%%%天若有情天亦老，我为长者续一秒+1s
path = [-13 0 0; -9 0 0;-11 0 0;-11 2 0;-11 -2.5 0;-11 -2.5 2;-7 4 2;...
    -7 4 0;-7 -4.5 0;2 4 2;2 4 0;0 5 0;-2 4 0;-3 3 0;-2 1 0;2 0 0;3 -2 0;2 -4 0;...
   0 -5 0;-2 -4 0; ];
% plot3(path(:,1),path(:,2),path(:,3),'color','k','LineWidth',2);
p = mstraj(path,[3 3 3],[],[],0.3,3);
% plot3(p(:,1),p(:,2),p(:,3))
Tp = transl(0.1*p);
Tp = homtrans(transl(0.5,0,0),Tp);
a=squeeze(Tp(1,4,:));
b=squeeze(Tp(2,4,:));
c=squeeze(Tp(3,4,:));
m=1;
for i=1:length(a)
    if c(i)~=0
        continue
    else
        a1(m)=a(i);
        b1(m)=b(i);
        c1(m)=c(i);
        m=m+1;
        
    end
end
a2=a1';
b2=b1';
c2=c1';
 plot3(a,b,c)
p560.tool =trotx(pi);
q = p560.ikine6s(Tp);
p560.plot(q)
 
% % % T1 = transl(0.4,0.2,0)* trotx(pi);
% % % T2 = transl(0.4,-0.2,0)* trotx(pi/2);
% % % q1 = p560.ikine6s(T1);
% % % q2 = p560.ikine6s(T2);
% % % t = [0:0.05:2];
% % % [q,qd,qdd] = mtraj(@tpoly,q1,q2,t);
% % % T = p560.fkine(q);
% % % p = transl(T);
% plot(p(:,1),p(:,2));
% plot(t,tr2rpy(T))
% plot(t,q)


