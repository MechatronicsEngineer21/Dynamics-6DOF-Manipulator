% %% DH and Inertia Parameters of robot
alpha = [pi/2,0,0,pi/2,-pi/2,0];
a = [0,0.410,.340,0,0,0];
d = [0.179,0,0,0.188,0.132,0];

% Masses from link-1 to end-effector
m = [4.043687892216430,4.088351020081470,2.975514330882000,2.005251413706830,1.580964077451070,0.312204788418883];

% Center of masses
p_c1 = [5.62863246619805e-06,	-0.0712754141025166,	-0.00583821731829928];
p_c2 = [0.266884576505637,	   6.57683680493905e-06,       0.145260394562513];
p_c3 = [0.237472823221729,	  -2.00826081189764e-06,	  0.0376164502968732];
p_c4 = [-0.00183715117493422,	-0.0365459234829408,	   0.172476350166275];
p_c5 = [0.00232838749144237,	 0.0384258837201684,	   0.123429863395004];
p_c6 = [0.000634869947333528,	0.00391637711202809,	   0.120490768542163];

% Inertia matrix 
% [Ixx,Iyy, Izz Tyz, Ixz, Ixy]
% 0.0256365144777215	0.00665469331160818	0.0250354565670247	-0.000723537076243379	1.82000648689342e-07	9.11011344785925e-07
In_1= [0.0256365144777215, 9.11011344785925e-07,1.82000648689342e-07;
9.11011344785925e-07, 0.00665469331160818,-0.000723537076243379;
1.82000648689342e-07, -0.000723537076243379, 	0.0250354565670247];

% 0.0934242825775859	0.429895593447408	0.343952026019285	-3.66364099206361e-06	-0.158696051806179	-7.49373409208807e-06
In_2= [0.0934242825775859, -7.49373409208807e-06, -0.158696051806179;
-7.49373409208807e-06, 0.429895593447408,-3.66364099206361e-06;
-0.158696051806179, -3.66364099206361e-06, 0.343952026019285];

% 0.00796511011804721	0.198641151365072	0.194685517188124	3.91874159242292e-07	-0.0258905573571414	-6.23480885898958e-07
In_3 = [0.00796511011804721, -6.23480885898958e-07, -0.0258905573571414;
-6.23480885898958e-07, 0.198641151365072, 3.91874159242292e-07;
-0.0258905573571414, 3.91874159242292e-07,0.194685517188124];

% 0.0643547214988473	0.0620233656652383	0.00427910029465841	0.0129916682948349	0.000632738514403877	-0.000137312641834570
In_4= [0.0643547214988473, -0.000137312641834570, 0.000632738514403877;
-0.000137312641834570, 0.0620233656652383, 0.0129916682948349;
0.000632738514403877,0.0129916682948349, 0.00427910029465841	];

% 0.0277827074304922	0.0257145715219778	0.00355361777275285	-0.00773249663034194	-0.000451742327448627	-0.000144088686954488
In_5= [0.0277827074304922, -0.000144088686954488, -0.000451742327448627;
-0.000144088686954488, 0.0257145715219778, -0.00773249663034194;
-0.000451742327448627, -0.00773249663034194, 0.00355361777275285];

% 0.00464887224718288	0.00470493042096833	0.000154844733533007	-0.000149950574946370	-2.44987020649305e-05	1.01430408305949e-05
In_6= [0.00464887224718288, 1.01430408305949e-05, -2.44987020649305e-05;
1.01430408305949e-05, 0.00470493042096833,-0.000149950574946370;
-2.44987020649305e-05, -0.000149950574946370, 0.000154844733533007];


% %% PARAMETERs AND SYMBOLs
g = 9.806650000000000;
alpha_0=alpha(1);alpha_1=alpha(2);alpha_2=alpha(3);alpha_3=alpha(4);alpha_4=alpha(5);alpha_5=alpha(6);
a_0=a(1);a_1=a(2);a_2=a(3);a_3=a(4);a_4=a(5);a_5=a(6);
d_1=d(1);d_2=d(2);d_3=d(3);d_4=d(4);d_5=d(5);d_6=d(6);
p_cx1=p_c1(1);p_cy1=p_c1(2);p_cz1=p_c1(3);
p_cx2=p_c2(1);p_cy2=p_c2(2);p_cz2=p_c2(3);
p_cx3=p_c3(1);p_cy3=p_c3(2);p_cz3=p_c3(3);
p_cx4=p_c4(1);p_cy4=p_c4(2);p_cz4=p_c4(3);
p_cx5=p_c5(1);p_cy5=p_c5(2);p_cz5=p_c5(3);
p_cx6=p_c6(1);p_cy6=p_c6(2);p_cz6=p_c6(3);
m_1=m(1);m_2=m(2);m_3=m(3);m_4=m(4);m_5=m(5);m_6=m(6);
q_1=sym('q_1');q_2=sym('q_2');q_3=sym('q_3');q_4=sym('q_4');q_5=sym('q_5');q_6=sym('q_6');
dq_1=sym('dq_1');dq_2=sym('dq_2');dq_3=sym('dq_3');dq_4=sym('dq_4');dq_5=sym('dq_5');dq_6=sym('dq_6');
ddq_1=sym('ddq_1');ddq_2=sym('ddq_2');ddq_3=sym('ddq_3');ddq_4=sym('ddq_4');ddq_5=sym('ddq_5');ddq_6=sym('ddq_6');
%% ROTATION MATRICEs
R_1= [cos(q_1), -sin(q_1)*cos(alpha_0), sin(q_1)*sin(alpha_0);
      sin(q_1), cos(q_1)*cos(alpha_0), -cos(q_1)*sin(alpha_0);
      0,         sin(alpha_0),           cos(alpha_0)];
R_2= [cos(q_2), -sin(q_2)*cos(alpha_1), sin(q_2)*sin(alpha_1);
      sin(q_2), cos(q_2)*cos(alpha_1), -cos(q_2)*sin(alpha_1);
      0,         sin(alpha_1),           cos(alpha_1)];
R_3=[cos(q_3), -sin(q_3)*cos(alpha_2), sin(q_3)*sin(alpha_2);
      sin(q_3), cos(q_3)*cos(alpha_2), -cos(q_3)*sin(alpha_2);
      0,         sin(alpha_2),           cos(alpha_2)];
R_4=[cos(q_4), -sin(q_4)*cos(alpha_3), sin(q_4)*sin(alpha_3);
      sin(q_4), cos(q_4)*cos(alpha_3), -cos(q_4)*sin(alpha_3);
      0,         sin(alpha_3),           cos(alpha_3)];
R_5= [cos(q_5), -sin(q_5)*cos(alpha_4), sin(q_5)*sin(alpha_4) ;
      sin(q_5), cos(q_5)*cos(alpha_4), -cos(q_5)*sin(alpha_4);
      0,         sin(alpha_4),           cos(alpha_4)];
R_6=[cos(q_6), -sin(q_6)*cos(alpha_5), sin(q_6)*sin(alpha_5) ;
      sin(q_6), cos(q_6)*cos(alpha_5), -cos(q_6)*sin(alpha_5);
      0,         sin(alpha_5),           cos(alpha_5)];
%% POSITION VECTORs
p_1=[a_0*cos(q_1);a_0*sin(q_1);d_1];
p_2=[a_1*cos(q_2);a_1*sin(q_2);d_2];
p_3=[a_2*cos(q_3);a_2*sin(q_3);d_3];
p_4=[a_3*cos(q_4);a_3*sin(q_4);d_4];
p_5=[a_4*cos(q_5);a_4*sin(q_5);d_5];
p_6=[a_5*cos(q_6);a_5*sin(q_6);d_6];

R_20=R_1*R_2;
R_30=R_20*R_3;
R_40=R_30*R_4;
R_50=R_40*R_5;
R_60=R_50*R_6;

%% SYSTEM's STATEs
q=[q_1;q_2;q_3;q_4;q_5;q_6];
dq=[dq_1;dq_2;dq_3;dq_4;dq_5;dq_6];
ddq=[ddq_1;ddq_2;ddq_3;ddq_4;ddq_5;ddq_6];
A1=[[R_1,p_1];0,0,0,1];
A2=[[R_2,p_2];0,0,0,1];
A3=[[R_3,p_3];0,0,0,1];
A4=[[R_4,p_4];0,0,0,1];
A5=[[R_5,p_5];0,0,0,1];
A6=[[R_6,p_6];0,0,0,1];
t12=(A1*A2);
t13=(t12*A3);
t14=(t13*A4);
t15=(t14*A5);
t16=(t15*A6);
z0=[0 0 1];
z1= A1(1:3,3)';
z2=t12(1:3,3)';
z3=t13(1:3,3)';
z4=t14(1:3,3)';
z5=t15(1:3,3)';
o0=[0 0 0];
o1= A1(1:3,4)';
o2=t12(1:3,4)';
o3=t13(1:3,4)';
o4=t14(1:3,4)';
o5=t15(1:3,4)';
o6=t16(1:3,4)';

jv1=[zeros(6,3)]';
jw1=[z0;zeros(5,3)]';
%----------------second link-------------- 
jv2=[cross(z0,(o1-o0));zeros(5,3)]';

jw2=[z0;z1;zeros(4,3)]';
%----------------third link--------------
jv3=[cross(z0,(o2-o0));cross(z1,(o2-o1));zeros(4,3)]';

jw3=[z0;z1;z2;zeros(3,3)]';
%----------------fourth link--------------
jv4=[cross(z0,(o3-o0));cross(z1,(o3-o1));cross(z2,(o3-o2));zeros(3,3)]';
jw4=[z0;z1;z2;z3;zeros(2,3)]';
%----------------fifth link--------------
jv5=[cross(z0,(o4-o0));cross(z1,(o4-o1));cross(z2,(o4-o2));...
    cross(z3,(o4-o3));zeros(2,3)]';
jw5=[z0;z1;z2;z3;z4;zeros(1,3)]';
%----------------sixth link--------------
jv6=[cross(z0,(o5-o0));cross(z1,(o5-o1));...
    cross(z2,(o5-o2));cross(z3,(o5-o3));cross(z4,(o5-o4));zeros(1,3)]';
jw6=[z0;z1;z2;z3;z4;z5]';
 
j= [jw6;jv6];
% ROBOT's INERTIA (MASS) MATRIX
M =  jv6'*m_6*jv6 + jw6'*R_60*In_6*R_60'*jw6 ...
    +jv5'*m_5*jv5 + jw5'*R_50*In_5*R_50'*jw5 ...
    +jv4'*m_4*jv4 + jw4'*R_40*In_4*R_40'*jw4 ...
    +jv3'*m_3*jv3 + jw3'*R_30*In_3*R_30'*jw3 ...
    +jv2'*m_2*jv2 + jw2'*R_20*In_2*R_20'*jw2 ...
    +jv1'*m_1*jv1 + jw1'* R_1*In_1* R_1'*jw1;


% CORIOLIS and CENTRIFUGAL MATRIX
C = sym(zeros(6,6));
for k=1:6
   for s=1:6
      C(k,s)=.5*((diff(M(k,s),q_1)+diff(M(k,1),q(s,1))-diff(M(1,s),q(k,1)))*dq_1...
                +(diff(M(k,s),q_2)+diff(M(k,2),q(s,1))-diff(M(2,s),q(k,1)))*dq_2...
                +(diff(M(k,s),q_3)+diff(M(k,3),q(s,1))-diff(M(3,s),q(k,1)))*dq_3...
                +(diff(M(k,s),q_4)+diff(M(k,4),q(s,1))-diff(M(4,s),q(k,1)))*dq_4...
                +(diff(M(k,s),q_5)+diff(M(k,5),q(s,1))-diff(M(5,s),q(k,1)))*dq_5...
                +(diff(M(k,s),q_6)+diff(M(k,6),q(s,1))-diff(M(6,s),q(k,1))))*dq_6;
   end
end

%% POTENTIAL ENERGIES and GRAVITY VECTOR
pc1_base = o0 + eye(3,3) * p_c1';
pc2_base = o1 + R_20 * p_c2';
pc3_base = o2 + R_30 * p_c3';
pc4_base = o3 + R_40 * p_c4';
pc5_base = o4 + R_50 * p_c5';
pc6_base = o5 + R_60 * p_c6';

% Compute potential energy U = sum(m_i * g * h_i)
U = m_1 * g * pc1_base(3,3) + ...
    m_2 * g * pc2_base(3,3) + ...
    m_3 * g * pc3_base(3,3) + ...
    m_4 * g * pc4_base(3,3) + ...
    m_5 * g * pc5_base(3,3) + ...
    m_6 * g * pc6_base(3,3);

G1 = diff(U, q_1);
G2 = diff(U, q_2);
G3 = diff(U, q_3);
G4 = diff(U, q_4);
G5 = diff(U, q_5);
G6 = diff(U, q_6);

% Assemble the gravity vector
G = [G1; G2; G3; G4; G5; G6];


q_val=   [0.1;0.2;0.3;0.4;0.5;0.6];
dq_val = [0.01; 0.02; 0.03; 0.04; 0.05; 0.06];  
ddq_val = [0.001; 0.002; 0.003; 0.004; 0.005; 0.006];

% Numerical evaluation of Jacobian, Mass (M), Coriolis (C), and Gravity (G) 
% matrices for torque computation at given joint positions and velocities
Jac_value=subs(j, q,q_val);
Jac_value=double(Jac_value);
M_value=subs(M, q,q_val);
M_value=double(M_value);
C_val=C*dq_val;
C_value=subs(C_val, q,q_val);
C_value=subs(C_value, dq,dq_val);
C_value=double(C_value);
G_val = subs(G, q, q_val);
G_val=double(G_val);

torque = M*ddq + C*dq + G;
torque_val = subs(torque, q, q_val);
torque_val = subs(torque_val, dq, dq_val);
torque_val = subs(torque_val, ddq, ddq_val);
torque_val=double(torque_val);

%% Separate torques for each joint
tau1 = torque(1);
tau2 = torque(2);
tau3 = torque(3);
tau4 = torque(4);
tau5 = torque(5);
tau6 = torque(6);

% You cam import six joint torques by the below six lines code 
% matlabFunction(tau1,'File', 'torque1.m', 'Vars', {q,dq, ddq})
% matlabFunction(tau2,'File', 'torque2.m', 'Vars', {q,dq, ddq})
% matlabFunction(tau3,'File', 'torque3.m', 'Vars', {q,dq, ddq})
% matlabFunction(tau4,'File', 'torque4.m', 'Vars', {q,dq, ddq})
% matlabFunction(tau5,'File', 'torque5.m', 'Vars', {q,dq, ddq})
% matlabFunction(tau6,'File', 'torque6.m', 'Vars', {q,dq, ddq})
