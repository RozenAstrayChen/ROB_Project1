%% 1 Forward Kinematics
clc;clear;
ap = [-90 /180 * pi,0,90 /180 * pi,-90 /180 * pi,90 /180 * pi,0];
a = [0,0.432,-0.02,0,0,0];
d = [0,0,0.149,0.433,0,0];
%theta = [(rand*2-1)*160 /180*pi,(rand*2-1)*125 /180*pi,(rand*2-1)*135 /180*pi,(rand*2-1)*140 /180*pi,(rand*2-1)*100 /180*pi,(rand*2-1)*260 /180*pi];
theta = [50,50,50,50,50,50]/180*pi
A =zeros(4,4,6);
T = [0,0,0,0];
for i = 1:6
    A(:,:,i) = [cos(theta(i)),-sin(theta(i)).*cos(ap(i)),sin(theta(i)).*sin(ap(i)),a(i).*cos(theta(i));
        sin(theta(i)),cos(theta(i)).*cos(ap(i)),-cos(theta(i)).*sin(ap(i)), a(i).*sin(theta(i));
        0, sin(ap(i)), cos(ap(i)),d(i);
        0,0,0,1];
end
Tn = A(:,:,1)*A(:,:,2)*A(:,:,3)*A(:,:,4)*A(:,:,5)*A(:,:,6)
x = Tn(1,4);
y = Tn(2,4);
z = Tn(3,4);
phi = atan2(Tn(2,3),Tn(1,3)) / pi*180 ;
tt = atan2(((Tn(1,3)^2+Tn(2,3)^2)^0.5),Tn(3,3))/ pi*180;
psi = atan2(Tn(3,2)/sin(tt),-Tn(3,1)/sin(tt)) / pi*180;
fprintf("(x, y, z) = (%.2f, %.2f, %.2f) m\n",x,y,z)
fprintf("(phi, theta, psi) = (%.2f, %.2f, %.2f) degree\n",phi,tt,psi)


%% 2 Inverse Kinematics
%clc;clear;

d3 = 0.149; d4 = 0.433;
a2 = 0.432; a3 = -0.02;

px = Tn(1,4); ax = Tn(1,3); ox = Tn(1,2); nx = Tn(1,1);
py = Tn(2,4); ay = Tn(2,3); oy = Tn(2,2); ny = Tn(2,1);
pz = Tn(3,4); az = Tn(3,3); oz = Tn(3,2); nz = Tn(3,1);
p1 = (px^2 + py^2 )^0.5;
p3 = (d4^2 + a3^2 )^0.5;
M = (px^2 + py^2 + pz^2 -a2^2 - a3^2 -d4^2 -d3^2)/(2*a2);

theta1a = (atan2(py,px)-atan2(d3/p1, (1-(d3/p1)^2)^0.5)) 
theta1b = (atan2(py,px)-atan2(d3/p1,-(1-(d3/p1)^2)^0.5)) 
theta3a = (-atan2(a3,d4)+atan2(M/p3,(1-(M/p3)^2)^0.5))
theta3b = (-atan2(a3,d4)+atan2(M/p3,-(1-(M/p3)^2)^0.5))
theta2aa  = atan2(-(a3+a2*cos(theta3a))*pz+(cos(theta1a)*px+sin(theta1a)*py)*(a2*sin(theta3a)+d4)...
    ,(d4+a2*sin(theta3a))*pz+(cos(theta1a)*px+sin(theta1a)*py)*(a2*cos(theta3a)+a3))-theta3a;
theta2ab  = atan2(-(a3+a2*cos(theta3b))*pz+(cos(theta1a)*px+sin(theta1a)*py)*(a2*sin(theta3b)+d4)...
    ,(d4+a2*sin(theta3b))*pz+(cos(theta1a)*px+sin(theta1a)*py)*(a2*cos(theta3b)+a3))-theta3b;
theta2ba  = atan2(-(a3+a2*cos(theta3a))*pz+(cos(theta1b)*px+sin(theta1b)*py)*(a2*sin(theta3a)+d4)...
    ,(d4+a2*sin(theta3a))*pz+(cos(theta1b)*px+sin(theta1b)*py)*(a2*cos(theta3a)+a3))-theta3a;
theta2bb  = atan2(-(a3+a2*cos(theta3b))*pz+(cos(theta1b)*px+sin(theta1b)*py)*(a2*sin(theta3b)+d4)...
    ,(d4+a2*sin(theta3b))*pz+(cos(theta1b)*px+sin(theta1b)*py)*(a2*cos(theta3b)+a3))-theta3b;

theta4aap = atan2(-ax*sin(theta1a)+cos(theta1a)*ay,ax*cos(theta1a)*cos(theta2aa+theta3a)+sin(theta1a)*cos(theta2aa+theta3a)*ay-sin(theta2aa+theta3a)*az)
theta4aan = atan2(ax*sin(theta1a)-cos(theta1a)*ay,-ax*cos(theta1a)*cos(theta2aa+theta3a)-sin(theta1a)*cos(theta2aa+theta3a)*ay+sin(theta2aa+theta3a)*az)
theta4abp = atan2(-ax*sin(theta1a)+cos(theta1a)*ay,ax*cos(theta1a)*cos(theta2ab+theta3b)+sin(theta1a)*cos(theta2ab+theta3b)*ay-sin(theta2ab+theta3b)*az)
theta4abn = atan2(ax*sin(theta1a)-cos(theta1a)*ay,-ax*cos(theta1a)*cos(theta2ab+theta3b)-sin(theta1a)*cos(theta2ab+theta3b)*ay+sin(theta2ab+theta3b)*az)
theta4bap = atan2(-ax*sin(theta1b)+cos(theta1b)*ay,ax*cos(theta1b)*cos(theta2ba+theta3a)+sin(theta1b)*cos(theta2ba+theta3a)*ay-sin(theta2ba+theta3a)*az)
theta4ban = atan2(ax*sin(theta1b)-cos(theta1b)*ay,-ax*cos(theta1b)*cos(theta2ba+theta3a)-sin(theta1b)*cos(theta2ba+theta3a)*ay+sin(theta2ba+theta3a)*az)
theta4bbp = atan2(-ax*sin(theta1b)+cos(theta1b)*ay,ax*cos(theta1b)*cos(theta2bb+theta3b)+sin(theta1b)*cos(theta2bb+theta3b)*ay-sin(theta2bb+theta3b)*az)
theta4bbn = atan2(ax*sin(theta1b)-cos(theta1b)*ay,-ax*cos(theta1b)*cos(theta2bb+theta3b)-sin(theta1b)*cos(theta2bb+theta3b)*ay+sin(theta2bb+theta3b)*az)

theta6aan = atan2(-cos(theta1a)*sin(theta2aa+theta3a)*ox-sin(theta1a)*sin(theta2aa+theta3a)*oy-cos(theta2aa+theta3a)*oz...
    ,cos(theta1a)*sin(theta2aa+theta3a)*nx+sin(theta1a)*sin(theta2aa+theta3a)*ny+cos(theta2aa+theta3a)*nz)
theta6aap = atan2(cos(theta1a)*sin(theta2aa+theta3a)*ox+sin(theta1a)*sin(theta2aa+theta3a)*oy+cos(theta2aa+theta3a)*oz...
    ,-cos(theta1a)*sin(theta2aa+theta3a)*nx-sin(theta1a)*sin(theta2aa+theta3a)*ny-cos(theta2aa+theta3a)*nz)
theta6abn = atan2(-cos(theta1a)*sin(theta2ab+theta3b)*ox-sin(theta1a)*sin(theta2ab+theta3b)*oy-cos(theta2ab+theta3b)*oz...
    ,cos(theta1a)*sin(theta2ab+theta3b)*nx+sin(theta1a)*sin(theta2ab+theta3b)*ny+cos(theta2ab+theta3b)*nz)
theta6abp = atan2(cos(theta1a)*sin(theta2ab+theta3b)*ox+sin(theta1a)*sin(theta2ab+theta3b)*oy+cos(theta2ab+theta3b)*oz...
    ,-cos(theta1a)*sin(theta2ab+theta3b)*nx-sin(theta1a)*sin(theta2ab+theta3b)*ny-cos(theta2ab+theta3b)*nz)
theta6ban = atan2(-cos(theta1b)*sin(theta2ba+theta3a)*ox-sin(theta1b)*sin(theta2ba+theta3a)*oy-cos(theta2ba+theta3a)*oz...
    ,cos(theta1b)*sin(theta2ba+theta3a)*nx+sin(theta1b)*sin(theta2ba+theta3a)*ny+cos(theta2ba+theta3a)*nz)
theta6bap = atan2(cos(theta1b)*sin(theta2ba+theta3a)*ox+sin(theta1b)*sin(theta2ba+theta3a)*oy+cos(theta2ba+theta3a)*oz...
    ,-cos(theta1b)*sin(theta2ba+theta3a)*nx-sin(theta1b)*sin(theta2ba+theta3a)*ny-cos(theta2ba+theta3a)*nz)
theta6bbn = atan2(-cos(theta1b)*sin(theta2bb+theta3b)*ox-sin(theta1b)*sin(theta2bb+theta3b)*oy-cos(theta2bb+theta3b)*oz...
    ,cos(theta1b)*sin(theta2bb+theta3b)*nx+sin(theta1b)*sin(theta2bb+theta3b)*ny+cos(theta2bb+theta3b)*nz)
theta6bbp = atan2(cos(theta1b)*sin(theta2bb+theta3b)*ox+sin(theta1b)*sin(theta2bb+theta3b)*oy+cos(theta2bb+theta3b)*oz...
    ,-cos(theta1b)*sin(theta2bb+theta3b)*nx-sin(theta1b)*sin(theta2bb+theta3b)*ny-cos(theta2bb+theta3b)*nz)

theta5aap = atan2((cos(theta1a)*cos(theta4aap)*cos(theta2aa+theta3a)-sin(theta1a)*sin(theta4aap))*ax + (cos(theta1a)*...
    sin(theta4aap)+cos(theta4aap)*cos(theta2aa+theta3a)*sin(theta1a))*ay-cos(theta4aap)*sin(theta2aa+theta3a)*az,cos(theta1a)*sin(theta2aa+theta3a)*ax+sin(theta1a)*sin(theta2aa+theta3a)*ay+cos(theta2aa+theta3a)*az)
theta5aan = atan2((cos(theta1a)*cos(theta4aan)*cos(theta2aa+theta3a)-sin(theta1a)*sin(theta4aan))*ax + (cos(theta1a)*...
    sin(theta4aan)+cos(theta4aan)*cos(theta2aa+theta3a)*sin(theta1a))*ay-cos(theta4aan)*sin(theta2aa+theta3a)*az,cos(theta1a)*sin(theta2aa+theta3a)*ax+sin(theta1a)*sin(theta2aa+theta3a)*ay+cos(theta2aa+theta3a)*az)
theta5abp = atan2((cos(theta1a)*cos(theta4abp)*cos(theta2ab+theta3b)-sin(theta1a)*sin(theta4abp))*ax + (cos(theta1a)*...
    sin(theta4abp)+cos(theta4abp)*cos(theta2ab+theta3b)*sin(theta1a))*ay-cos(theta4abp)*sin(theta2ab+theta3b)*az,cos(theta1a)*sin(theta2ab+theta3b)*ax+sin(theta1a)*sin(theta2ab+theta3b)*ay+cos(theta2ab+theta3b)*az)
theta5abn = atan2((cos(theta1a)*cos(theta4abn)*cos(theta2ab+theta3b)-sin(theta1a)*sin(theta4abn))*ax + (cos(theta1a)*...
    sin(theta4abn)+cos(theta4abn)*cos(theta2ab+theta3b)*sin(theta1a))*ay-cos(theta4abn)*sin(theta2ab+theta3b)*az,cos(theta1a)*sin(theta2ab+theta3b)*ax+sin(theta1a)*sin(theta2ab+theta3b)*ay+cos(theta2ab+theta3b)*az)
theta5bap = atan2((cos(theta1b)*cos(theta4bap)*cos(theta2ba+theta3a)-sin(theta1b)*sin(theta4bap))*ax + (cos(theta1b)*...
    sin(theta4bap)+cos(theta4bap)*cos(theta2ba+theta3a)*sin(theta1b))*ay-cos(theta4bap)*sin(theta2ba+theta3a)*az,cos(theta1b)*sin(theta2ba+theta3a)*ax+sin(theta1b)*sin(theta2ba+theta3a)*ay+cos(theta2ba+theta3a)*az)
theta5ban = atan2((cos(theta1b)*cos(theta4ban)*cos(theta2ba+theta3a)-sin(theta1b)*sin(theta4ban))*ax + (cos(theta1b)*...
    sin(theta4ban)+cos(theta4ban)*cos(theta2ba+theta3a)*sin(theta1b))*ay-cos(theta4ban)*sin(theta2ba+theta3a)*az,cos(theta1b)*sin(theta2ba+theta3a)*ax+sin(theta1b)*sin(theta2ba+theta3a)*ay+cos(theta2ba+theta3a)*az)
theta5bbp = atan2((cos(theta1b)*cos(theta4bbp)*cos(theta2bb+theta3b)-sin(theta1b)*sin(theta4bbp))*ax + (cos(theta1b)*...
    sin(theta4bbp)+cos(theta4bbp)*cos(theta2bb+theta3b)*sin(theta1b))*ay-cos(theta4bbp)*sin(theta2bb+theta3b)*az,cos(theta1b)*sin(theta2bb+theta3b)*ax+sin(theta1b)*sin(theta2bb+theta3b)*ay+cos(theta2bb+theta3b)*az)
theta5bbn = atan2((cos(theta1b)*cos(theta4bbn)*cos(theta2bb+theta3b)-sin(theta1b)*sin(theta4bbn))*ax + (cos(theta1b)*...
    sin(theta4bbn)+cos(theta4bbn)*cos(theta2bb+theta3b)*sin(theta1b))*ay-cos(theta4bbn)*sin(theta2bb+theta3b)*az,cos(theta1b)*sin(theta2bb+theta3b)*ax+sin(theta1b)*sin(theta2bb+theta3b)*ay+cos(theta2bb+theta3b)*az)

sol1 =[theta1a theta2aa theta3a theta4aap theta5aap theta6aap]/pi*180
judgeangle(sol1);
sol2 =[theta1a theta2ab theta3b theta4abp theta5abp theta6abp]/pi*180
judgeangle(sol2);
sol3 =[theta1a theta2aa theta3a theta4aan theta5aan theta6aan]/pi*180
judgeangle(sol3);
sol4 =[theta1a theta2ab theta3b theta4abn theta5abn theta6abn]/pi*180
judgeangle(sol4);
sol5 =[theta1b theta2ba theta3a theta4bap theta5bap theta6bap]/pi*180
judgeangle(sol5);
sol6 =[theta1b theta2bb theta3b theta4bbp theta5bbp theta6bbp]/pi*180
judgeangle(sol6);
sol7 =[theta1b theta2ba theta3a theta4ban theta5ban theta6ban]/pi*180
judgeangle(sol7);
sol8 =[theta1b theta2bb theta3b theta4bbn theta5bbn theta6bbn]/pi*180
judgeangle(sol8);
