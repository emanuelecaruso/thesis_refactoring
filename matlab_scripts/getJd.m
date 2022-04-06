clear
clc

syms A B C D E F G H
syms u1 v1 invd1 delta_invd1

%%

% A*u1*invd1 + B*v1*invd1 + C*invd1 + D
% ----------------------------
% E*u1*invd1 + F*v1*invd1 + G*invd1 + H

eq = ( A*u1*(1/(invd1+delta_invd1)) + B*v1*(1/(invd1+delta_invd1)) + C*(1/(invd1+delta_invd1)) + D ) / ( E*u1*(1/(invd1+delta_invd1)) + F*v1*(1/(invd1+delta_invd1)) + G*(1/(invd1+delta_invd1)) + H )
du = simplify(subs(diff(eq,delta_invd1),delta_invd1,0))

