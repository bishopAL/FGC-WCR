dataRecord = [];
dataForceRecord = [];
% iList=0.03:-0.002:-0.03;
S_ = 500;
for wholeLope=1:100
    %input: Fg; A=A*Si;
    %parameters: L1; L2; L3; lengthHf; widthHf
    % i1 = iList(wholeLope);
    i1 = -0.03 + (0.03 - (-0.03)) * rand;
    i2 = -0.03 + (0.03 - (-0.03)) * rand;
    i3 = -0.03 + (0.03 - (-0.03)) * rand;
    i4 = -0.03 + (0.03 - (-0.03)) * rand;
    i5 = -0.01 + (0.01 - (-0.01)) * rand;
    i6 = -0.01 + (0.01 - (-0.01)) * rand;
    i7 = -0.01 + (0.01 - (-0.01)) * rand;
    i8 = -0.01 + (0.01 - (-0.01)) * rand;

    Fg=[0 0 -9.8*0.487];  %×ã¶ËÊÜµœµÄºÏÍâÁŠ  0.487*9.8= -4.77
    
    upd=5;      %µ¥ÍÈÉÏÏÞ
    downd=-5; %µ¥ÍÈÏÂÏÞ
    
    L1=0.060; L2=0.060; L3=0.030; lengthHf=0.055; widthHf=0.030;
    Pfc=[  L2+lengthHf+i1      L1+widthHf+i5     -L3
               L2+lengthHf+i2      -L1-widthHf+i6   -L3
               -L2-lengthHf+i3    L1+widthHf+i7     -L3
               -L2-lengthHf+i4    -L1-widthHf+i8   -L3];    
     %Ñ¡ÔñŸØÕó£¬°Ú¶¯ÍÈ 0-LF 1-RF 2-LH 3-RH
    S0=[zeros(3,3)    zeros(3,9)% LF in swing phase
            zeros(9,3)    eye(9,9)];
    S1=[eye(3,3)    zeros(3,9)
            zeros(3,12)
            zeros(6,6)    eye(6,6)];
    S2=[eye(3,3)        zeros(3,9)
             zeros(3,3)    eye(3,3) zeros(3,6)
            zeros(3,12)
            zeros(3,9)    eye(3,3)];
    S3=[eye(3,3)    zeros(3,9)
            zeros(6,3) eye(6,6) zeros(6,3)
            zeros(3,12)    ];
        
    A1=[1 0 0 1 0 0 1 0 0 1 0 0
              0 1 0 0 1 0 0 1 0 0 1 0
              0 0 1 0 0 1 0 0 1 0 0 1];
    A2=[0  Pfc(1,3) -Pfc(1,2)  0  Pfc(2,3) -Pfc(2,2) 0  Pfc(3,3) -Pfc(3,2)   0  Pfc(4,3) -Pfc(4,2)
          -Pfc(1,3) 0 Pfc(1,1)      -Pfc(2,3) 0 Pfc(2,1)    -Pfc(3,3) 0 Pfc(3,1)    -Pfc(4,3) 0 Pfc(4,1)
          -Pfc(1,2) -Pfc(1,1)  0  -Pfc(2,2) -Pfc(2,1) 0   -Pfc(3,2) -Pfc(3,1) 0  -Pfc(4,2) -Pfc(4,1) 0];%²æ³ËŸØÕó
    A=[A1; A2];
    %%
    A=A*S0; %input Ñ¡Ôñ°Ú¶¯ÍÈ
    B=[Fg(1) Fg(2) Fg(3) zeros(1, 3)]';
    
    S=diag(linspace(S_,S_,6));%ÈšÖØ 200
    W=diag(linspace(1,1,12));%ÈšÖØ
    %W(7,7)=20;W(8,8)=20;W(9,9)=20;
    a=1;    %5
    H=2.*(A.'*S*A+a.*W);
    g=-2.*A.'*S*B;
    
    options = optimoptions('quadprog','MaxIter',500);%µüŽúŽÎÊý
    xQD=quadprog(H,g,[ ],[ ],[ ],[ ],linspace(downd,downd,12),linspace(upd,upd,12),zeros(1,12),options)
    
    Fc= [xQD(1) xQD(2) xQD(3) 
            xQD(4) xQD(5) xQD(6)
            xQD(7) xQD(8) xQD(9)
            xQD(10) xQD(11) xQD(12)];
    F=[0 0 0];
    T=[0 0 0];
    for i = 1:1:4
        F = F + Fc(i,:);
        T = T + cross(Fc(i,:), Pfc(i,:));      % {a}×ø±êÏµ
    end 
    Fc = -Fc;
    
    %单腿解算控制角度
    %input: targetPos,   F , footNum
    %parameters: L1; L2; L3; K_tq
    resCmd = zeros(4,3);
    for footNum=1:3
        syms q1 q2 q3 qx qy qz real % L1 L2 L3
        syms OriQ1 OriQ2 OriQ3  real
        syms theta5 d5 alpha4 a4
        syms theta7 d7 alpha6 a6
        syms theta9 d9 alpha8 a8
        assume(qx>-pi & qx<pi);
        assume(qy>-pi & qy<pi);
        assume(qz>-pi & qz<pi);
        
        % footNum = 1;
        %L1=0.045; L2=0.050; L3=0.022;% m
        L1=0.06; L2=0.060; L3=0.030;% m
        
        switch (footNum)
            case 0    %LF
                L1offset= pi/2;
                L4offset= -pi/2;
           %      F = [     0.0003
           %  0.0004
           % -0.5395];
                   F = Fc(1,:)';
                targetPosX= L2; 
                targetPosY= L1; 
                targetPosZ= -L3;
            case 1      %RF
                L1offset= -pi/2;
                L4offset= pi/2;
           %      F = [   0.0006
           %  0.0004
           % -2.0449];
                F = Fc(2,:)';
                targetPosX= L2; 
                targetPosY= -L1; 
                targetPosZ= -L3;
            case 2      %LH
                L1offset= pi/2;
                L4offset= pi/2;
           %      F = [  -0.0000
           %  0.0000
           % -0.1774];
           F = Fc(3,:)';
        %         F = [     0.0003
        %     0.0007
        %    -2.1751];
                targetPosX= -L2; 
                targetPosY= L1; 
                targetPosZ= -L3;
            case 3      %RH
                L1offset= -pi/2;
                L4offset= -pi/2;
                % F = [   -0.0006
                %     0.0004
                %    -2.0449];  
                F = Fc(4,:)';
                targetPosX= -L2; 
                targetPosY= -L1; 
                targetPosZ= -L3;
        end
        %足端受到的外力
        % F = [ 0
        %          0
        %    -1.1910];
        
        %%
        K_qt = diag([-0.001, -0.001, -0.00]);   %被动关节
        % K_tq = diag([-890, -890, -890, -890, -890, -890, -5e3, -5e1, -5e1]);%连杆和驱动关节
        % K_tq = diag([-600, -800, -6022, -71661, -573295, -48180, -6e2, -6e2, -3e4]);
        %K_tq = diag([-230, -420, -6022, -71661, -573295, -48180, -3e4, -3e4, -3e4]);
        K_tq = diag([-401.8, -218.6, -516.2, -302.7, -973.5, -881.4, -3e4, -3e4, -3e4]);
        
        syms theta d alpha a
        Tab=[            cos(theta),           -sin(theta),           0,             a
            cos(alpha)*sin(theta), cos(alpha)*cos(theta), -sin(alpha), -d*sin(alpha)
            sin(alpha)*sin(theta), sin(alpha)*cos(theta),  cos(alpha),  d*cos(alpha)
            0,                     0,           0,             1];
        
        Rx=[    1           0               0               0
                    0       cos(qx)     -sin(qx)   0
                    0       sin(qx)     cos(qx)     0
                    0           0                0              1];
        Ry=[cos(qy)        0        sin(qy)     0
                    0                1          0               0
            -sin(qy)          0        cos(qy)      0
                    0               0           0               1];
        Rz=[cos(qz) -sin(qz)   0               0
                sin(qz) cos(qz)     0               0
                0               0              1               0
                0               0              0               1];
        %LF
        T01=subs(Tab,{theta,d,a,alpha},{L1offset,0,0,0});   %
        T12=subs(Tab,{theta,d,a,alpha},{q1,0,0,pi/2});   
        T23=subs(Tab,{theta,d,a,alpha},{q2,0,0,-pi/2});   
        T34=subs(Tab,{theta,d,a,alpha},{L4offset,0,L1,0});   %
        T45=subs(Tab,{theta,d,a,alpha},{(q3+theta5) d5 a4 alpha4});
        T56=subs(Tab,{theta,d,a,alpha},{pi/2,0,L2,-pi/2});   
        T67=subs(Tab,{theta,d,a,alpha},{theta7 d7 a6 alpha6});   
        T78=subs(Tab,{theta,d,a,alpha},{0,0,L3,-pi/2});
        T89=subs(Tab,{theta,d,a,alpha},{theta9 d9 a8 alpha8}); 
        
        T09= T01 * T12 * T23 * T34 * T45 * T56 * T67 * T78 * T89;
        T=subs(T09,{theta5 d5 alpha4 a4 theta7 d7 alpha6 a6 theta9 d9 alpha8 a8},{0 0 0 0 0 0 0 0 0 0 0 0 });
        
        % 
        % [LFth0,LFth1,LFth2]= InverseKinematics([targetPosX; targetPosY; targetPosZ]);
        % T_tar=subs(T,{q1 q2 q3},{LFth0 LFth1 LFth2});
        % phi_z = atan2(T_tar(2,1),T_tar(1,1));
        % phi_y = atan2(-T_tar(3,1),T_tar(1,1)*cos(phi_z)+T_tar(2,1)*sin(phi_z));
        % phi_x = atan2(-T_tar(2,3)*cos(phi_z)+T_tar(1,3)*sin(phi_z),T_tar(2,2)*cos(phi_z)-T_tar(1,2)*sin(phi_z));
        % Tau=K_qt*[phi_x; phi_y; phi_z];
        % Tau=[0; 0; 0];
        % LFth0=0;LFth1=0;LFth2=0;
        
        syms t1z t1x t2z t2x t3z t3x real
        T_tq=subs(T09,{theta5 d5 alpha4 a4 theta7 d7 alpha6 a6 theta9 d9 alpha8 a8},{atan(t1x/L1) t1z atan(t1z/L1) t1x atan(t2x/L2) t2z atan(t2z/L2) t2x atan(t3x/L3) t3z atan(t3z/L3) t3x });
        g_tq=T_tq*(Rz*Ry*Rx);
        
        % phi_z = atan2(g_tq(2,1),g_tq(1,1));
        % phi_y = atan2(-g_tq(3,1),g_tq(1,1)*cos(phi_z)+g_tq(2,1)*sin(phi_z));
        % phi_x = atan2(-g_tq(2,3)*cos(phi_z)+g_tq(1,3)*sin(phi_z),g_tq(2,2)*cos(phi_z)-g_tq(1,2)*sin(phi_z));
        % eval(subs([phi_x phi_y phi_z],{qx,qy,qz},{0.1,0.2,0.3}))
        
        g_xyz=g_tq(1:3,4);
        jacobian11 = diff(g_xyz(1), t1z);
        jacobian12 = diff(g_xyz(1), t1x);
        jacobian13 = diff(g_xyz(1), t2z);
        jacobian14 = diff(g_xyz(1), t2x);
        jacobian15 = diff(g_xyz(1), t3z);
        jacobian16 = diff(g_xyz(1), t3x);
        
        jacobian21 = diff(g_xyz(2), t1z);
        jacobian22 = diff(g_xyz(2), t1x);
        jacobian23 = diff(g_xyz(2), t2z);
        jacobian24 = diff(g_xyz(2), t2x);
        jacobian25 = diff(g_xyz(2), t3z);
        jacobian26 = diff(g_xyz(2), t3x);
        
        jacobian31 = diff(g_xyz(3), t1z);
        jacobian32 = diff(g_xyz(3), t1x);
        jacobian33 = diff(g_xyz(3), t2z);
        jacobian34 = diff(g_xyz(3), t2x);
        jacobian35 = diff(g_xyz(3), t3z);
        jacobian36 = diff(g_xyz(3), t3x);
        
        % jacobian41 = diff(phi_x, t1z);
        % jacobian42 = diff(phi_x, t1x);
        % jacobian43 = diff(phi_x, t2z);
        % jacobian44 = diff(phi_x, t2x);
        % jacobian45 = diff(phi_x, t3z);
        % jacobian46 = diff(phi_x, t3x);
        % jacobian51 = diff(phi_y, t1z);
        % jacobian52 = diff(phi_y, t1x);
        % jacobian53 = diff(phi_y, t2z);
        % jacobian54 = diff(phi_y, t2x);
        % jacobian55 = diff(phi_y, t3z);
        % jacobian56 = diff(phi_y, t3x);
        % jacobian61 = diff(phi_z, t1z);
        % jacobian62 = diff(phi_z, t1x);
        % jacobian63 = diff(phi_z, t2z);
        % jacobian64 = diff(phi_z, t2x);
        % jacobian65 = diff(phi_z, t3z);
        % jacobian66 = diff(phi_z, t3x);
        % 
        % jacobian_tt = [jacobian41 jacobian42 jacobian43 jacobian44 jacobian45 jacobian46
        %                       jacobian51 jacobian52 jacobian53 jacobian54 jacobian55 jacobian56
        %                       jacobian61 jacobian62 jacobian63 jacobian64 jacobian65 jacobian66];
        % jacobian_ttt= jacobian_tt(1:3,1:3);
        
        jacobian_t = [  jacobian11 jacobian12 jacobian13 jacobian14 jacobian15 jacobian16
                      jacobian21 jacobian22 jacobian23 jacobian24 jacobian25 jacobian26
                      jacobian31 jacobian32 jacobian33 jacobian34 jacobian35 jacobian36];
        %               jacobian41 jacobian42 jacobian43 jacobian44 jacobian45 jacobian46
        %               jacobian51 jacobian52 jacobian53 jacobian54 jacobian55 jacobian56
        %               jacobian61 jacobian62 jacobian63 jacobian64 jacobian65 jacobian66];
                  
        jacobian11 = diff(g_xyz(1), q1);
        jacobian12 = diff(g_xyz(1), q2);
        jacobian13 = diff(g_xyz(1), q3);
        jacobian14 = diff(g_xyz(1), qx);
        jacobian15 = diff(g_xyz(1), qy);
        jacobian16 = diff(g_xyz(1), qz);
        
        jacobian21 = diff(g_xyz(2), q1);
        jacobian22 = diff(g_xyz(2), q2);
        jacobian23 = diff(g_xyz(2), q3);
        jacobian24 = diff(g_xyz(2), qx);
        jacobian25 = diff(g_xyz(2), qy);
        jacobian26 = diff(g_xyz(2), qz);
        
        jacobian31 = diff(g_xyz(3), q1);
        jacobian32 = diff(g_xyz(3), q2);
        jacobian33 = diff(g_xyz(3), q3);
        jacobian34 = diff(g_xyz(3), qx);
        jacobian35 = diff(g_xyz(3), qy);
        jacobian36 = diff(g_xyz(3), qz);
        
        
        % jacobian41 = diff(phi_x, q1);
        % jacobian42 = diff(phi_x, q2);
        % jacobian43 = diff(phi_x, q3);
        % jacobian44 = diff(phi_x, qx);
        % jacobian45 = diff(phi_x, qy);
        % jacobian46 = diff(phi_x, qz);
        % jacobian51 = diff(phi_y, q1);
        % jacobian52 = diff(phi_y, q2);
        % jacobian53 = diff(phi_y, q3);
        % jacobian54 = diff(phi_y, qx);
        % jacobian55 = diff(phi_y, qy);
        % jacobian56 = diff(phi_y, qz);
        % jacobian61 = diff(phi_z, q1);
        % jacobian62 = diff(phi_z, q2);
        % jacobian63 = diff(phi_z, q3);
        % jacobian64 = diff(phi_z, qx);
        % jacobian65 = diff(phi_z, qy);
        % jacobian66 = diff(phi_z, qz);
        % 
        % jacobian_qq = [jacobian41 jacobian42 jacobian43 jacobian44 jacobian45 jacobian46
        %                       jacobian51 jacobian52 jacobian53 jacobian54 jacobian55 jacobian56
        %                       jacobian61 jacobian62 jacobian63 jacobian64 jacobian65 jacobian66];
        % jacobian_qqq= jacobian_qq(1:3,1:3);
        % eval(subs(jacobian_qqq*[1;1;1], {q1 q2 q3},{0.1, 0.2, 0.3}))
        
        jacobian_q = [  jacobian11 jacobian12 jacobian13 jacobian14 jacobian15 jacobian16
                      jacobian21 jacobian22 jacobian23 jacobian24 jacobian25 jacobian26
                      jacobian31 jacobian32 jacobian33 jacobian34 jacobian35 jacobian36];        
                  
             %连杆和旋转关节 均含弹性形变      力矩     
        %  jacobian_tq = [jacobian_t jacobian_q; jacobian_tt jacobian_qq;];
        % Jt=eval(jacobian_tq.' * [F;0;0;0]);
        % enq1=Jt==diag( [-890, -890, -890, -890, -890, -890, -5e3, -5e1, -5e1, -0.001, -0.001, -0.00])*[t1z; t1x; t2z; t2x; t3z; t3x; q1-OriQ1; q2-OriQ2; q3-OriQ3; qx; qy; qz ];      
        % enq2=g_xyz==[targetPosX; targetPosY; targetPosZ];%target pos
        % varsT = [t1z t1x t2z t2x t3z t3x q1 q2 q3 OriQ1 OriQ2 OriQ3 qx qy qz];
        % varsT = vpasolve(enq1, enq2, varsT);
                  
                  %连杆和旋转关节 均含弹性形变
        jacobian_tq = [jacobian_t jacobian_q(:,1:3)];
        Jt=eval(jacobian_tq.' * F);
        enq1=Jt==K_tq*[t1z; t1x; t2z; t2x; t3z; t3x; q1-OriQ1; q2-OriQ2; q3-OriQ3 ];      
        enq2=g_xyz==[targetPosX; targetPosY; targetPosZ];%target pos
        varsT = [t1z t1x t2z t2x t3z t3x q1 q2 q3 OriQ1 OriQ2 OriQ3];
        varsT = vpasolve(enq1, enq2, varsT);
        
                  %连杆 含弹性形变，旋转关节为刚性
        %input {q1 q2 q3} {0 0 0}
        % Jt=eval(subs(jacobian_t.' * F,{q1 q2 q3},{LFth0 LFth1 LFth2}));
        % enq1=Jt==K_tq(1:6,1:6)*[t1z; t1x; t2z; t2x; t3z; t3x];      %{q1 q2 q3},{0 0 0}的K_t
        % varsT = [t1z t1x t2z t2x t3z t3x];
        % varsT = vpasolve(enq1, varsT);
        
        % g_xyz_t=subs(g_xyz,{t1z t1x t2z t2x t3z t3x},{varsT.t1z varsT.t1x varsT.t2z varsT.t2x varsT.t3z varsT.t3x });
        % enq2=g_xyz_t==[targetPosX; targetPosY; targetPosZ];%target pos
        % varsQ = [q1 q2 q3];
        % varsQ = vpasolve(enq2, varsQ);
        % 
        % q1r=varsQ.q1
        % q2r=varsQ.q2
        % q3r=varsQ.q3
        q1r=varsT.q1
        q2r=varsT.q2
        q3r=varsT.q3
        t1zr=varsT.t1z 
        t1xr=varsT.t1x 
        t2zr=varsT.t2z 
        t2xr=varsT.t2x 
        t3zr=varsT.t3z 
        t3xr=varsT.t3x
        CmdTh0r = varsT.OriQ1
        CmdTh1r = varsT.OriQ2
        CmdTh2r = varsT.OriQ3
        
        xyzr_cmd =eval(subs(g_xyz,{q1 q2 q3 t1z t1x t2z t2x t3z t3x},{CmdTh0r CmdTh1r CmdTh1r 0 0 0 0 0 0}))
        % xyzr_q =eval(subs(g_xyz,{q1 q2 q3 t1z t1x t2z t2x t3z t3x},{q1r q2r q3r 0 0 0 0 0 0})) ;%不含连杆形变,只含主动旋转关节扭簧
        % xyzr =eval(subs(g_xyz,{q1 q2 q3 t1z t1x t2z t2x t3z t3x},{q1r q2r q3r t1zr t1xr t2zr t2xr t3zr t3xr})) 
        % deltaXYZ = xyzr_cmd - xyzr
        % T_tq_r =eval(subs(g_tq,{q1 q2 q3 t1z t1x t2z t2x t3z t3x qx qy qz},{q1r q2r q3r t1zr t1xr t2zr t2xr t3zr t3xr 0 0 0 }));
        resCmd(footNum+1,:)=xyzr_cmd';
    end
    
    
    
    % " T_tq_r==eye(4)*Rz*Ry*Rx "
    % phi_z = atan2(T_tq_r(2,1),T_tq_r(1,1));
    % phi_y = atan2(-T_tq_r(3,1),T_tq_r(1,1)*cos(phi_z)+T_tq_r(2,1)*sin(phi_z));
    % phi_x = atan2(-T_tq_r(2,3)*cos(phi_z)+T_tq_r(1,3)*sin(phi_z),T_tq_r(2,2)*cos(phi_z)-T_tq_r(1,2)*sin(phi_z));
    % eval(subs(T_tq_r*(Rz*Ry*Rx).',{qx qy qz},{phi_x phi_y phi_z}))
    dataRecord = [dataRecord; [reshape(Pfc(2:end,1:2),1,6), resCmd(2,3), resCmd(3,3), resCmd(4,3)]];
    % dataForceRecord = [dataForceRecord; [i1, Fc(2,3), Fc(3,3), Fc(4,3)]];
end
save("trainingData.mat",'dataRecord');
% save("compNewForce"+num2str(S_)+".mat",'dataForceRecord');