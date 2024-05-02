function [Pfcbr] = comp(p1, p2, p3, p4, p5, p6)
format short;
digits(5);
syms fc0x fc0y fc0z fc1x fc1y fc1z fc2x fc2y fc2z fc3x fc3y fc3z real;
Fc = [ fc0x fc0y fc0z 
       fc1x fc1y fc1z 
       fc2x fc2y fc2z       
       fc3x fc3y fc3z ];
        
syms Rx Ry Rz Px Pyy Pz real;
assume(Rx>-pi & Rx<pi);
assume(Ry>-pi & Ry<pi);
assume(Rz>-pi & Rz<pi);

syms factor_x  factor_y factor_z factor_s12  factor_s0 th0 th1 th2 real;  
% syms LFth0 LFth1 LFth2 RFth0 RFth1 RFth2 LHth0 LHth1 LHth2 RHth0 RHth1 RHth2 real;


% Px = 0.00092819;
% Pyy = 0.00063006;
% Pz = 0.00876;
% Rx = -0.057788;
% Ry = 0.050479;
% Rz = 0.016208;

Rotx=[  1      0         0   0
        0 cos(Rx)  -sin(Rx)  0
        0 sin(Rx)   cos(Rx)  0
        0      0         0   1];
        
Roty=[cos(Ry)   0 sin(Ry) 0
           0    1      0  0
     -sin(Ry)   0 cos(Ry) 0 
           0    0      0  1];
        
Rotz=[cos(Rz) -sin(Rz)  0 0
      sin(Rz)  cos(Rz)  0 0
            0       0   1 0
            0       0    0 1]; 
Tp = [Px Pyy Pz];
Tab = Rotx * Roty * Rotz * [eye(3) Tp.'; 0 0 0 1]; 

G=[0 0 4.9];     %  0.5kg  世界坐标系{w}与调整后的平行机身质心坐标系相同，即新坐标系{a}；调整前机身质心坐标系为旧坐标系{b}
Fg=[0 0 -4.9];  %足端受到的外力    % input
PcOrib=[0 0 0 1];
Pca=[0 0 0 1];
L1=0.045; L2=0.050; L3=0.022;  lengthHF=0.035; widthHF=0.03;% m
%L1=0.05; L2=0.047; L3=0.040;  length2=0.06; width2=0.03;% m

% K of motor
K0 = diag([-5e-1, -5e-1, -5e-1]);
K1 = diag([-5e-1, -5e-1, -5e-1]);
K2 = diag([-5e-1, -5e-1, -5e-1]);
K3 = diag([-5e-1, -5e-1, -5e-1]);
K=[-5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1, -5e-1];

Pfs0 = [  L2    L1 -L3
               L2  -L1 -L3
             -L2    L1 -L3
             -L2  -L1 -L3];     
Pscb = [  lengthHF    widthHF 0
                 lengthHF  -widthHF 0
               -lengthHF    widthHF 0
               -lengthHF  -widthHF 0];
Psca = [  lengthHF    widthHF 0
                 lengthHF  -widthHF 0
               -lengthHF    widthHF 0
               -lengthHF  -widthHF 0];           
Pcb = zeros(4,3);
PfcOrib = Pfs0 + Pscb + Pcb;   

% input
Pfca =  [  0.0850,    0.0750,   -0.0220;
            p1,    p2,   -0.0220;
                p3,  p4,   -0.0220;
               p5,  p6,   -0.0220;
               ];
% Pfca =  [  0.0850    0.0750   -0.0220
%                 0.0850   -0.0750   -0.0220
%                -0.0850    0.0750   -0.0220
%                -0.0850   -0.0750   -0.0220];
% Pfca =   [  0.086076,  0.074115,  -0.02208
%                 0.084086, -0.075622, -0.030733
%                 -0.083685,  0.076867, -0.030658
%                 -17/200,     -3/40,   -11/500];
Pfsa = Pfca - Psca;


% PcOria = PcOrib * (Tab.')^(-1);
% temp = [1; 1; 1; 1];
% Psa4X4 = [Pscb temp] * (Tab.')^(-1);  
% Psa = Psa4X4(1:4,1:3);
% PfsOria1 = Pfca - Psa;
% temp1 =zeros(4,1);              %%点在坐标系间变换，与矢量在坐标系间变换的不同
% PfsOrib4X4 = [PfsOria1 temp1] * Tab.';      
% PfsOrib1 = PfsOrib4X4(1:4,1:3)
%PfcOri = PfsOria + Psca;

temp = [1; 1; 1; 1];
PfcOrib4 = [Pfca temp] * Tab.';
PfcOrib = PfcOrib4(1:4,1:3);
PfsOrib = PfcOrib - Pscb;

PfcOria4X4 = [PfcOrib temp] * (Tab.')^(-1);  
PfcOria = PfcOria4X4(1:4,1:3);

% inverse Kinematics
syms factor_yc factor_xc  factor_zc factor_x  factor_y x y z real;  
jointCmdPos0 = factor_xc * (asin(L3 / sqrt( z*z + y*y )) + atan2(z,factor_y * y) );     
jointCmdPos1 = factor_yc * (asin((y * y + x * x + z * z + L1 * L1 - L2 * L2 - L3 * L3) / ( 2 * L1 * sqrt (y * y +  x * x + z * z - L3 * L3))) - atan2(sqrt(y * y + z * z - L3 * L3) , factor_x * x));
jointCmdPos2 = factor_zc * asin((L1 * L1 + L2 * L2 + L3 * L3 - y * y - x * x - z * z) / (2 * L1 * L2));

PfsaT = Pfsa';
LFth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(1), PfsaT(2), PfsaT(3), 1,-1,-1,1,1});
LFth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(1), PfsaT(2), PfsaT(3), 1,-1,-1,1,1});
LFth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(1), PfsaT(2), PfsaT(3), 1,-1,-1,1,1});

RFth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(4), PfsaT(5), PfsaT(6), -1,1,1,1,-1});
RFth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(4), PfsaT(5), PfsaT(6), -1,1,1,1,-1});
RFth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(4), PfsaT(5), PfsaT(6), -1,1,1,1,-1});

LHth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(7), PfsaT(8), PfsaT(9), 1,-1,-1,-1,1});
LHth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(7), PfsaT(8), PfsaT(9), 1,-1,-1,-1,1});
LHth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(7), PfsaT(8), PfsaT(9), 1,-1,-1,-1,1});

RHth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(10), PfsaT(11), PfsaT(12), -1,1,1,-1,-1});
RHth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(10), PfsaT(11), PfsaT(12), -1,1,1,-1,-1});
RHth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsaT(10), PfsaT(11), PfsaT(12), -1,1,1,-1,-1});
% eval([LFth0 LFth1 LFth2])
% eval([RFth0 RFth1 RFth2])
% eval([LHth0 LHth1 LHth2])
% eval([RHth0 RHth1 RHth2])

PfsOriT = PfsOrib';
OriLFth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(1), PfsOriT(2), PfsOriT(3), 1,-1,-1,1,1});
OriLFth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(1), PfsOriT(2), PfsOriT(3), 1,-1,-1,1,1});
OriLFth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(1), PfsOriT(2), PfsOriT(3), 1,-1,-1,1,1});

OriRFth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(4), PfsOriT(5), PfsOriT(6), -1,1,1,1,-1});
OriRFth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(4), PfsOriT(5), PfsOriT(6), -1,1,1,1,-1});
OriRFth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(4), PfsOriT(5), PfsOriT(6), -1,1,1,1,-1});

OriLHth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(7), PfsOriT(8), PfsOriT(9), 1,-1,-1,-1,1});
OriLHth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(7), PfsOriT(8), PfsOriT(9), 1,-1,-1,-1,1});
OriLHth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(7), PfsOriT(8), PfsOriT(9), 1,-1,-1,-1,1});

OriRHth0 = subs(jointCmdPos0, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(10), PfsOriT(11), PfsOriT(12), -1,1,1,-1,-1});
OriRHth1 = subs(jointCmdPos1, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(10), PfsOriT(11), PfsOriT(12), -1,1,1,-1,-1});
OriRHth2 = subs(jointCmdPos2, {x, y, z, factor_xc, factor_yc, factor_zc, factor_x, factor_y  }, {PfsOriT(10), PfsOriT(11), PfsOriT(12), -1,1,1,-1,-1});
% eval([OriLFth0 OriLFth1 OriLFth2])
% eval([OriRFth0 OriRFth1 OriRFth2])
% eval([OriLHth0 OriLHth1 OriLHth2])
% eval([OriRHth0 OriRHth1 OriRHth2])

syms TH0 TH1 TH2 TH3 real; 
TH0 = [LFth0 LFth1 LFth2] - [OriLFth0 OriLFth1 OriLFth2];
TH1 = [RFth0 RFth1 RFth2] - [OriRFth0 OriRFth1 OriRFth2];
TH2 = [LHth0 LHth1 LHth2] - [OriLHth0 OriLHth1 OriLHth2];
TH3 = [RHth0 RHth1 RHth2] - [OriRHth0 OriRHth1 OriRHth2];
% TH0 = [LFth0 LFth1 LFth2] - [0 0 0];
% TH1 = [RFth0 RFth1 RFth2] - [0 0 0];
% TH2 = [LHth0 LHth1 LHth2] - [0 0 0];
% TH3 = [0 0 0];

%forward Kinematics
footToShoulderPos0 = factor_x * (-factor_s12 * L1 * sin(th1) + L2 * cos(th1 + th2));
footToShoulderPos1 = factor_y * ((( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * cos(th0) + factor_s0 * L3 * sin(th0)));
footToShoulderPos2 = factor_z * (( L1 * cos(th1) + factor_s12 * L2 * sin(th1 + th2) ) * factor_s0 * sin(th0) -  L3 * cos(th0));
footToShoulder = [footToShoulderPos0 footToShoulderPos1 footToShoulderPos2];

% footToShoulderLF = subs(footToShoulder, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {LFth0, LFth1, LFth2, 1,1,1,1,1});
% footToShoulderRF = subs(footToShoulder, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {RFth0, RFth1, RFth2, 1,-1,1,-1,-1});
% footToShoulderLH = subs(footToShoulder, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {LHth0, LHth1, LHth2, -1,1,1,1,1});
% footToShoulderRH = subs(footToShoulder, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {RHth0, RHth1, RHth2, -1,-1,1,-1,-1});
% Pfs = [footToShoulderLF; footToShoulderRF; footToShoulderLH; footToShoulderRH];
% Pfc = Pfs + Pscb;

jacobian11 = diff(footToShoulderPos0, th0);
jacobian12 = diff(footToShoulderPos0, th1);
jacobian13 = diff(footToShoulderPos0, th2);
jacobian21 = diff(footToShoulderPos1, th0);
jacobian22 = diff(footToShoulderPos1, th1);
jacobian23 = diff(footToShoulderPos1, th2);
jacobian31 = diff(footToShoulderPos2, th0);
jacobian32 = diff(footToShoulderPos2, th1);
jacobian33 = diff(footToShoulderPos2, th2);
jacobian = [  jacobian11 jacobian12 jacobian13
              jacobian21 jacobian22 jacobian23
              jacobian31 jacobian32 jacobian33];
          
jacobianLF = subs(jacobian, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {LFth0, LFth1, LFth2, 1,1,1,1,1});
jacobianRF = subs(jacobian, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {RFth0, RFth1, RFth2, 1,-1,1,-1,-1});
jacobianLH = subs(jacobian, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {LHth0, LHth1, LHth2, -1,1,1,1,1});
jacobianRH = subs(jacobian, {th0, th1, th2, factor_x, factor_y, factor_z, factor_s12, factor_s0  }, {RHth0, RHth1, RHth2, -1,-1,1,-1,-1});

    
Fc(1, :) = (jacobianLF^(-1).'*K0*TH0.').';
Fc(2, :) = (jacobianRF^(-1).'*K1*TH1.').';
Fc(3, :) = (jacobianLH^(-1).'*K2*TH2.').';
Fc(4, :) = (jacobianRH^(-1).'*K3*TH3.').';

% Fc(1, :) = [0 0 0 ];
Fc(4, :) = [0 0 0 ];
        
% Fw4x4 = [Fc  ones(4, 1)] *(Rotx * Roty * Rotz).';
% Fw = Fw4x4(1:4,1:3);
% Pcw = Pcc * Tcw.';

F=[0 0 0];
T=[0 0 0];
for i = 1:1:4
    F = F + Fc(i,:);
    T = T + cross(Fc(i,:), Pfca(i,:));      % {a}坐标系
end 
% eval(F)
% eval(T)

eqn1 = F+Fg == zeros(1, 3);
eqn2 = T  == zeros(1, 3); % + cross(Fg,  Pcw(1, 1:3))=0
 
vars = [Rx Ry Rz Px Pyy Pz];
%vars = [LFth0 LFth1 LFth2 RFth0 RFth1 RFth2 LHth0 LHth1 LHth2 ];  
% vars = [Px Pyy Pz OriLFth0 OriLFth1 OriLFth2 OriRFth0 OriRFth1 OriRFth2 OriLHth0 OriLHth1 OriLHth2 OriRHth0 OriRHth1 OriRHth2];% Rx Ry Rz
vars = vpasolve(eqn1, eqn2, vars, zeros(1, 6));  % 
Px = vars.Px
Pyy = vars.Pyy
Pz = vars.Pz
Rx = vars.Rx;
Ry = vars.Ry;
Rz = vars.Rz;
Tabr = eval(subs(Tab,{Rx, Ry, Rz, Px, Pyy, Pz},[Rx, Ry, Rz, Px, Pyy, Pz])) ;
Pfcbr = [Pfca temp] * Tabr.'
end

