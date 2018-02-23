clc
clear all
%Akinlawon Solomon
% Camera Calibration

%% ================================== Select 10 Points According to Grid ===========================
P1=[0;0;0];
P2=[0;50;0];
P3=[0;25;25];
P4=[0;50;50];
P5=[50;0;50];
P6=[50;50;0];
P7=[0;0;50];
P8=[50;0;0];
P9=[25;25;0];
P10=[25;0;25];

p=[2320 1670;2819 2176;2544 1167;2931 304;1045 420;972 2309; 2358 448; 1108 1726; 1814 1914; 1730 1090];
P=[[P1 P2 P3 P4 P5 P6 P7 P8 P9 P10]' ones(10,1)];

Pr=zeros(20,12);
for i=1:10
  Pr(i,:)=[P(i,:) zeros(1,4)  -p(i,1)*P(i,:)];
  Pr(2*i,:)=[zeros(1,4) P(i,:)  -p(i,2)*P(i,:)];
end

%% ================================ Calculate Eigenvectors ============================================
[P_mat,D]=eig(Pr'*Pr);
[J,I] = min(max(D,[],2));
M=P_mat(:,I);
M=[M(1:4,1)';M(5:8,1)';M(9:12,1)'];

%%
% %% Extrinsic and Intrinsic Parameters (Optional)
% 
% a1=M(1,1:3);
% a2=M(2,1:3);
% a3=M(3,1:3);
% 
% phi=1/norm(a1);
% r3=phi*a3;
% x0=phi^2*dot(a1,a3);
% y0=phi^2*dot(a2,a3);
% 
% theta=-(dot(cross(a1,a3),cross(a2,a3)))/(norm(cross(a1,a3))*norm(cross(a2,a3)));
% 
% theta=acos(theta);
% alpha=phi^2*norm(cross(a1,a3))*sin(theta);
% beta=phi^2*norm(cross(a2,a3))*sin(theta);
% 
% K=[alpha -alpha/tan(theta) x0; 0 beta/sin(theta) y0; 0  0  1];
% 
% r1=cross(a2,a3)/norm(cross(a2,a3));
% r2=cross(r3,r1);
% 
% t=phi*inv(K)*M(:,4);
% %% M with parameters calculated
% L=[alpha*r1-(alpha/tan(theta))*r2+x0*r3  alpha*t(1)-(alpha/tan(theta))*t(2)+x0*t(3)];
% O=[beta/sin(theta)*r2+y0*r3      beta/sin(theta)*t(2)+y0*t(3)];
% Q=[r3 t(3)];
% 
% M_pr=[L;O;Q];

%% ============== 3D to 2D Conversion =========================
for i=1:size(P,1)    
calib_2D(i,:)=[dot(M(1,:),P(i,:))/dot(M(3,:),P(i,:))  dot(M(2,:),P(i,:))/dot(M(3,:),P(i,:))];
end

%% ================ Calibration for all points =========================
vec = {5*[0:1:10],5*[0:1:10], [0] };

n = numel(vec);
c = cell(1,n); 
[c{end:-1:1}] = ndgrid(vec{end:-1:1});
c = cat(n+1, c{:});
c = reshape(c,[],n);

xy=c;
Ry=[0 0 1; 0 1 0; -1 0 0];  % Rotate matrix by 90 degrees so xy points become yz points.
Rx=[1 0 0; 0 0 1; 0 -1 0];  % '' '' so xy points become xz points.
yz=c*Ry;
yz=sortrows(yz,2);
xz=c*Rx;
Ap=[xy;yz;xz];


%% ====================== Finding xy's for each point ======================
Ap=[Ap ones(363,1)];

for i=1:size(Ap,1)
Pstore(i,:)=[dot(M(1,:),Ap(i,:))/dot(M(3,:),Ap(i,:))  dot(M(2,:),Ap(i,:))/dot(M(3,:),Ap(i,:))];
end
%% ======================= Show Calculated Points on Grid ============================
 imshow('project_image2.jpe')
  grid on
  hold on

scatter(calib_2D(:,1),calib_2D(:,2),'yellow','filled');
Pstore2=Pstore(1:121,:);
Pstore3=Pstore(122:242,:);
Pstore4=Pstore(243:363,:);
for i=1:11
    hold on
    plot(Pstore2(i:11:111+(i-1),1), Pstore2(i:11:111+(i-1),2),'-r.')
    plot(Pstore2(11*i-10:11*i,1), Pstore2(11*i-10:11*i,2),'-r.')
    
    plot(Pstore3(i:11:111+(i-1),1), Pstore3(i:11:111+(i-1),2),'-b.')
    plot(Pstore3(11*i-10:11*i,1), Pstore3(11*i-10:11*i,2),'-b.')
    
    plot(Pstore4(i:11:111+(i-1),1), Pstore4(i:11:111+(i-1),2),'-g.')
    plot(Pstore4(11*i-10:11*i,1), Pstore4(11*i-10:11*i,2),'-g.')
end

%% Error calculation between known points and predicted points
error=mean(abs(calib_2D-p));


