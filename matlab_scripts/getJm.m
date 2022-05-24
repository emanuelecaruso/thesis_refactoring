clear
clc

syms fx fy d1 d2 w h u1 v1 u2 v2
syms r00 r01 r02 r10 r11 r12 r20 r21 r22 t0 t0 t1 t2
syms slope1 slope2 vh

k=[fx  0 w/2 ;
   0  fy h/2;
   0  0 1       ];

kinv = inv(k);

T= [r00 r01 r02 t0;
    r10 r11 r12 t1;
    r20 r12 r22 t2;
    0 0 0 1];

syms dx1 dx2 dx3 dx4 dx5 dx6 real % perturbation (to be estimated)

dt= [dx1, dx2, dx3, dx4, dx5, dx6];
dT=v2T(dt);  % perturbation


% p_proj1 = [u1*d1;v1*d1;d1]; %uv and depth on cam1 (d is known)
p_proj2 = [u2*d2;v2*d2;d2]; %uv and depth on cam2 (d2 is unknown)


a=[kinv*p_proj2;1]; %3D point in cam2 frame + 1 for homogeneous transf
% b=[kinv*p_proj1;1]; %3D point in cam1 frame + 1 for homogeneous transf

syms pb0 pb1 pb2
b=[pb0; pb1; pb2; 1];

mlt=dT*b; %T transform cam1 to cam2 (cam1 expressed in cam2)


% d2 function as d1
disp("d2")
expr1 = mlt(3) == a(3);
slv_d2 = solve(expr1,d2);
pretty(simplify(slv_d2,'Steps',1000));

%%

% u2=f(u1,v1,d1) (get coord as u2)
disp("u2")
expr2 = mlt(1) == a(1);
slv_u2 = solve(expr2,u2);
slv_u2 = subs(slv_u2,d2,slv_d2);
JMu = jacobian(slv_u2,[dx1,dx2,dx3,dx4,dx5,dx6]);
JMu = subs(JMu,[dx1,dx2,dx3,dx4,dx5,dx6],[0,0,0,0,0,0]);
JMu = simplify(JMu,'Steps',1000);
JMu = collect(JMu,[pb0, pb1, pb2]);

[N,D] = numden(JMu(6));
N= collect(N,[pb0, pb1, pb2])
D= collect(D,[pb0, pb1, pb2])
CN = coeffs(N,[pb0, pb1, pb2])
CD = coeffs(D,[pb0, pb1, pb2])
size(CN)
size(CD)

%%

% disp("v2")
expr3 = mlt(2) == a(2);
slv_v2 = solve(expr3,v2);
slv_v2 = subs(slv_v2,d2,slv_d2);
JMv = jacobian(slv_v2,[dx1,dx2,dx3,dx4,dx5,dx6]);
JMv = subs(JMv,[dx1,dx2,dx3,dx4,dx5,dx6],[0,0,0,0,0,0]);
JMv = simplify(JMv,'Steps',1000);
JMv = collect(JMv,[pb0, pb1, pb2])
[N,D] = numden(JMv(4));
N= collect(N,[pb0, pb1, pb2])
D= collect(D,[pb0, pb1, pb2])
CN = coeffs(N,[pb0, pb1, pb2])
CD = coeffs(D,[pb0, pb1, pb2])
size(CN)
size(CD)

%%

function T = v2T(v)
    t1_0=v(1);
    t2_0=v(2);
    t3=v(3);
    a1=v(4);
    a2=v(5);
    a3=v(6);
    c1=cos(a1);
    c2=cos(a2);
    c3=cos(a3);
    s1=sin(a1);
    s2=sin(a2);
    s3=sin(a3);
    
        R=[ c2*c3, -c2*s3, s2;
            c1*s3 + c3*s1*s2, c1*c3 - s1*s2*s3, -c2*s1;
            s1*s3 - c1*c3*s2, c3*s1 + c1*s2*s3,  c1*c2 ];
    
%     R=[ c1*c3+s1*s2*s3, c3*s1*s2-c1*s3, c2*s1;
%         c2*s3,  c2*c3, -s2;
%         c1*s2*s3-c3*s1, c1*c3*s2+s1*s3, c1*c2];
    t=[t1_0;t2_0;t3];
    T=[R,t;0,0,0,1];
    
end