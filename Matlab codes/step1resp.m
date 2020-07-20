%robot parameters
mb = 1.0184; %mass of robot
mw = 0.037; %mass of wheels
jb = 0.0061023; %moment of inertia about the centre of mass
r = 0.0325; %radius of wheels
jw = 3125E-05; %moment of inertia for the wheels
l = 0.05715; %distance from wheel axle to CoM
ke = 0.3819;
km = 0.4304;
R = 7.2; %motor resistance
b = 0.002; %Viscous friction constant
g = 9.81; %gravity
alp = (2*(R*b - ke*km)*(mb*l*l + mb*r*l +jb))/ R*(2*(jb*jw + jw*l*l*mb +jb*mw*r*r + l*l*mb*mw*r*r)+jb*mb*r*r);
bet = (-l*l*mb*mb*g*r*r)/(jb*(2*jw + mb*r*r + 2*mw*r*r) + 2*jw*l*l*mb +2*l*l*mb*mw*r*r);
gam = (-2*(R*b -ke*km)*(2*jw + mb*r*r + 2*mw*r*r + l*mb*r))/(R*r*(2*(jb*jw+ jw*l*l*mb + jb*mw*r*r + l*l*mb*mw*r*r)+jb*mb*r*r));
delt = (l*mb*g*(2*jw + mb*r*r + 2*mw*r*r))/(2*jb*jw + 2*jw*l*l*mb +jb*mb*r*r +2*jb*mw*r*r + 2*l*l*mb*mw*r*r);
chi = (km*r)/(R*b - ke*km);
A = [0 1 0 0;
 0 alp bet -r*alp;
 0 0 0 1;
 0 gam delt -r*alp];
B = [0; alp*chi; 0; gam*chi];
C = [1 0 0 0;
 0 1 0 0;
 0 0 1 0;
 0 0 0 1]
D = [0;0;0;0]
Q=C'*C
[n,d]=ss2tf(A,B,C,D)
G = ss(A,B,C,D)
R = 1;
[K,S,e] = lqr(G,Q,R)
sys1=ss(A-B*K,B,C,D)
step(sys1);