clear
clc

% using quaternions

syms u_A v_A real % uv seen in frame A (knownk)
syms d_A real % depth belief in frame A (known)
syms f w h real % intrindic cam parameters (known)
syms dt1 dt2 dt3 da1 da2 da3 real % perturbation (to be estimated)

% syms t1 t2 t3 a1 a2 a3 real % current guess (known)
syms r00 r01 r02 r10 r11 r12 r20 r21 r22 t0 t1 t2 % current guess (known)

syms grad_u grad_v %intensity gradients of the image (known)

K=[f  0 w/2 ;
   0  f h/2 ;
   0  0 1   ];

Kinv = inv(K);


p_A = Kinv * [ u_A*d_A; v_A*d_A; d_A ]; % 3d point expressed in frame A
p_A_hom = [p_A;1];
p_A_hom = simplify(p_A_hom,'Steps',1000);

% v_guess = [t1, t2, t3, a1, a2, a3];
% B_T_A=v2T(v_guess);  % current guess
B_T_A= [r00 r01 r02 t0;
        r10 r11 r12 t1;
        r20 r12 r22 t2;
        0 0 0 1]; % curreng guess

v_d= [dt1, dt2, dt3, da1, da2, da3];
DX=v2T(v_d);  % perturbation

p_B_hom=DX*B_T_A*p_A_hom;
p_B_hom = simplify(p_B_hom,'Steps',1000);

p_B=p_B_hom(1:3);

uv_B_ = K*p_B;
uv_B_ = simplify(uv_B_,'Steps',1000)

uv_B=uv_B_/uv_B_(3);
uv_B=uv_B(1:2);

uv_B=simplify(uv_B,'Steps',1000)

%%
clc

uv_B= collect(uv_B,[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,dt1,dt2,dt3,da1,da2,da3])
[num,den] = numden(uv_B);
CN1 = coeffs(num(1),[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,dt1,dt2,dt3,da1,da2,da3])
CD1 = coeffs(den(1),[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,dt1,dt2,dt3,da1,da2,da3])
CN2 = coeffs(num(2),[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,dt1,dt2,dt3,da1,da2,da3])
CD2 = coeffs(den(2),[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2,dt1,dt2,dt3,da1,da2,da3])
%%

J = jacobian(uv_B,[dt1,dt2,dt3,da1,da2,da3]);
J_quat_X = subs(J,[dt1,dt2,dt3,da1,da2,da3],[0,0,0,0,0,0]);
J_quat_X = simplify(J_quat_X,'Steps',1000)

%%
clc

J_quat_X= collect(J_quat_X,[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2]);

syms A B C D E F G H I M N L O P Q R S T real
J_quat_X_ = subs(J_quat_X,[2*d_A*f, 2*d_A*u_A-d_A*w, 2*d_A*v_A-d_A*h],[A,B,C]);
J_quat_X_ = subs(J_quat_X_,[4*d_A*f^2, 4*d_A*f*v_A - 2*d_A*f*h, 4*d_A*f*u_A - 2*d_A*f*w],[D,E,F]);
J_quat_X_ = subs(J_quat_X_,[-4*f^3, - 4*d_A*v_A*f^2 + A*h*f, - 4*d_A*u_A*f^2 + A*w*f],[G,H,I]);
J_quat_X_ = subs(J_quat_X_,[8*d_A*f^2, 8*d_A^2*f*u_A - 4*d_A^2*f*w, 4*d_A^2*u_A^2 - 4*d_A^2*u_A*w + d_A^2*w^2],[L,M,N]);

[num,den] = numden(J_quat_X_);



for i=1:12
    disp("aooooooooooo")
    disp(i)
    CN = coeffs(num(i),[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2])
    CD = coeffs(den(i),[r00,r01,r02,r10,r11,r12,r20,r21,r22,t0,t1,t2])
%     pretty(CN)
%     pretty(CD)
end
% J_quat = collect(J_quat,[dt1,dt2,dt3,da1,da2,da3]);
% CN = coeffs(J_quat,[dt1,dt2,dt3,da1,da2,da3])


%%

function T = v2T(v)
    t1=v(1);
    t2=v(2);
    t3=v(3);
    x=v(4);
    y=v(5);
    z=v(6);
    s=sqrt(1-(x^2+y^2+z^4));

    
    R=[ 1-2*(y^2)-2*(z^2), 2*x*y-2*s*z, 2*x*z+2*s*y ;
        2*x*y+2*s*z, 1-2*(x^2)-2*(z^2), 2*(y*z)-2*(s*x);
        2*x*z-2*s*y, 2*y*z+2*s*x, 1-2*(x^2)-2*(y^2) ];
    t=[t1;t2;t3];
    T=[R,t;0,0,0,1];
    
end