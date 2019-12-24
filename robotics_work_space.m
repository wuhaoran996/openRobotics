clc
clear all


% a1 = 0.5; 
% a2 = 0.3; 
% a3 = 0.2;
% DH(1) = Link([0 0 a1 pi/2]);
% DH(2) = Link([0 a2 0 0]);
% DH(3) = Link([0 0 a3 0]);
% 
%  r = SerialLink(DH);
%  r.teach();
% hold on
% 
% N=3000;    %随机次数
% 
% limitmax_1 = 0.0;
% limitmin_1 = 90.0;
% limitmax_2 = -90.0;
% limitmin_2 = 90.0;
% limitmax_3 = -90.0;
% limitmin_3 = 90.0;
% 
% th1=(limitmin_1+(limitmax_1-limitmin_1)*rand(N,1))*pi/180; %关节1限制
% th2=(limitmin_2+(limitmax_2-limitmin_2)*rand(N,1))*pi/180; %关节2限制
% th3=(limitmin_3+(limitmax_3-limitmin_3)*rand(N,1))*pi/180; %关节3限制
% 
% 
% 
% q=[th1,th2,th3];
% 
% Mricx=r.fkine(q);
% 
% for i=1:N
%     P=Mricx(1,i).t;
%    x(i)=P(1,1);
%    y(i)=P(2,1);
%    z(i)=P(3,1);
% end
% plot3(x,y,z,'r.')
% xlabel('X');
% ylabel('Y');
% zlabel('Z');
% 
% 
% 

a1 = 0.5; 
a2 = 0.3; 
a3 = 0.2;

DH(1) = Link([0 0 a1 pi/2]);
DH(2) = Link([0 a2 a2 0]);
DH(3) = Link([0 a1 a3 0]);
th1 = (-pi/6:0.05:pi/6) ;
th2 = (-2*pi/3:0.05:2*pi/3);
th3 = (-pi/2:0.05:pi/2) ;
q = {th1,th2,th3};


% L = Link([Th d a alpha])
r = SerialLink(DH);
r.display()
r.teach()
hold on 
[~,n] = size(DH);


var = sym('q',[n 1]);
assume(var,'real')

% generate a grid of theta1 and theta2,3,4 values
[Q{1:numel(q)}] = ndgrid(q{:}); 
T = simplify(vpa(r.fkine(var),3));
Pos = T.tv;
x(var(:)) = Pos(1);
X = matlabFunction(x);
X = X(Q{:});
y(var(:)) = Pos(2);
Y = matlabFunction(y);
Y = Y(Q{:});
z(var(:)) = Pos(3);
Z = matlabFunction(z);
Z = Z(Q{:});


plot3(X(:),Y(:),Z(:),'r.')
xlabel('X')
ylabel('Y')
zlabel('Z')
