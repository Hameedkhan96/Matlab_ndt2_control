syms pw px py pz qw qx qy qz
pw=5
qw=8


% q = [1 0 1 0];
% r = [1 0.5 0.5 0.75];
% mult = quatmultiply(q, r)

p=[pw px py pz]

q=[qw qx qy qz]

% p=pw+px+py+pz
% 
% q=qw+qx+qy+qz
% % 
quatprod=quatmultiply(p,q)
%%quatprod = quatmultiply(q,r)