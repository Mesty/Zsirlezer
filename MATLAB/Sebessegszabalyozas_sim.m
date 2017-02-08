
% Sebessegszabalyozas szimulacioja

% A valtozoneveket atirni nem szabad, a Simulink modell ezeket hasznalja

% Felhasznalt valtozok:
% t : a meresi idopontok vektora (oszlopvektor)
% u : a bemenet (gerjesztes) idofuggvenye (oszlopvektor)
% y : a kimenet (valasz) idofuggvenye (oszlopvektor)
% (A program a bemenet es a kimenet mereseit azonos idopontunak tekinti)
% P1 : A System Identification Toolbox-szal identifikalt Process Model
% u_stat : statikus karakterisztikahoz tartozo gerjesztesek (oszlopvektor)
% y_stat : statikus karakterisztikahoz tartozo valaszok (oszlopvektor)
% (Az y_stat az u_stat fuggvenyeben mindenkepp monoton novekvo kell legyen
% es a 0-hoz tartozo erteknek 0-nak kell lennie)
% y_sat : teliteshez tartozo sebesseg

%% Workspace kiuritese

clear scrsz Ts
clear y_sim0 u_sat u_I y_sim1
clear K T
clear i sys
clear siminput T_sim
clear ALAPJEL_MEREDEKSEG_GYORSITASNAL ALAPJEL_MEREDEKSEG_LASSITASNAL BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL

%% Parameterek (valtozoneveket atirni nem szabad, csak az ertekeket)

Ts = 0.01; % Mintavetelezesi ido [s]
T_cl = 0.5; % Zart kor eloirt idoallandoja [s]

%% Meredekseg korlatozasok
ALAPJEL_MEREDEKSEG_GYORSITASNAL = 0;
ALAPJEL_MEREDEKSEG_LASSITASNAL = 0;
BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL = 0;
BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL = 0;

%% Egyeb parameterek

scrsz = get(groot,'ScreenSize'); % Kepernyomeret, hogy szepen meretezzen

%% Modellalkotas

% Identifikalt rendszer parametereinek tarolasa
K = P1.Kp;
T = P1.Tp1;
y_stat = y_stat/K;
u_sat = y_sat/K;
i = 1;
while (y_stat(i) == 0)
    i = i+1;
end
inv_u_stat = y_stat(i-1:end);
inv_y_stat = u_stat(i-1:end);

% Identifikalt rendszer szimulacioja
y_sim0 = lsim(P1,u,t);

% Identifikalt rendszer valaszanak abrazolasa es osszehasonlitasa a merttel
figure('Position',[1 1 scrsz(3) scrsz(4)],'Name','Modellillesztes','NumberTitle','off')
subplot(2,1,1);
plot(t,u,'r');
legend('u','Location','NorthWest');
title('Modellillesztés');
subplot(2,1,2);
plot(t,y);
hold on;
plot(t,y_sim0,'r','LineWidth',2);
legend('y','y_s_i_m_,_0','Location','NorthWest');
xlabel('t [s]');

% Bemenet es kimenet abrazolasa
figure('Position',[1 1 scrsz(3) scrsz(4)],'Name','GV','NumberTitle','off')
subplot(2,1,1);
plot(t,u,'r');
legend('u','Location','NorthWest');
subplot(2,1,2);
plot(t,y);
legend('y','Location','NorthWest');
xlabel('t [s]');

% Statikus karakterisztika abrazolasa
figure('Position',[1 1 scrsz(3) scrsz(4)],'Name','Statikus karakterisztika','NumberTitle','off')
plot(u_stat,y_stat*K,'.-','MarkerSize',10);
title('Statikus karakterisztika');

% Szimulacio statikus nemlinearitassal
% u' letrehozasa az u-bol
u = timeseries(u);
u.Time = u.Time*Ts;
sim('statikus_nemlinearitas');
u_I = timeseries(u_I.signals.values);
u_I.Time = u_I.Time*Ts;
sim('statikus_linearizalt');
u = u.Data;
u_I = u_I.Data;
y_sim1 = y_sim1.signals.values;

% Statikus linearizalas abrazolasa
figure('Position',[1 1 scrsz(3) scrsz(4)],'Name','Statikus linearizalas','NumberTitle','off')
subplot(2,1,1);
plot(t,u,'r');
hold on;
plot(t,y);
legend('u','y','Location','NorthWest');
xlabel('t [s]');
subplot(2,1,2);
plot(t,u_I,'r');
hold on;
plot(t,y);
legend('u''','y','Location','NorthWest');
xlabel('t [s]');

% Linearizalt modell identifikaciojanak abrazolasa
figure('Position',[1 1 scrsz(3) scrsz(4)],'Name','Linearizalt modell identifikacioja','NumberTitle','off')
subplot(2,1,1);
plot(t,u_I,'r');
hold on;
plot(t,y);
plot(t,y_sim1,'b','LineWidth',2);
legend('u''','y','y_s_i_m_,_1','Location','NorthWest');
xlabel('t [s]');
subplot(2,1,2);
plot(t,u,'r');
hold on;
plot(t,y);
plot(t,y_sim0,'r','LineWidth',2);
plot(t,y_sim1,'b','LineWidth',2);
legend('u','y','y_s_i_m_,_0','y_s_i_m_,_1','Location','NorthWest');
xlabel('t [s]');

% Rendszer diszkretizalasa
sys = idtf(P1);
P1d = c2d(sys,Ts,'zoh');

% Diszkret rendszermodell valasza
siminput = [zeros(round(1/Ts),1);200*ones(round(7/Ts),1);zeros(round(5/Ts),1)];
T_sim = size(siminput,1)*Ts;
siminput = timeseries(siminput);
siminput.Time = siminput.Time*Ts;
sim('diszkret_rendszermodell_valasza');
open_system('diszkret_rendszermodell_valasza/Scope')

% Mintaveteles PI szabalyozo
% Parameterek beallitasa
z_d = -P1d.Denominator(2);
K_C = 1/P1d.Numerator(2)*(1-exp(-Ts/T_cl));
% Szimulacio futtatasa
sim('mintaveteles_PI_szabalyozo');
open_system('mintaveteles_PI_szabalyozo/Scope1')
open_system('mintaveteles_PI_szabalyozo/Scope')

% Holtsav figyelmen kivul hagyasanak hatasa
clear siminput
siminput = [zeros(round(1/Ts),1);20*ones(round(7/Ts),1);zeros(round(5/Ts),1)];
siminput = timeseries(siminput);
siminput.Time = siminput.Time*Ts;
sim('holtsav_figyelmen_kivul_hagyasanak_hatasa');
open_system('holtsav_figyelmen_kivul_hagyasanak_hatasa/Scope4')
open_system('holtsav_figyelmen_kivul_hagyasanak_hatasa/Scope')

% Beavatkozo szerv telites hatasa
clear siminput
siminput = [zeros(round(1/Ts),1);200*ones(round(7/Ts),1);zeros(round(5/Ts),1)];
siminput = timeseries(siminput);
siminput.Time = siminput.Time*Ts;
sim('beavatkozo_szerv_telites_hatasa');
open_system('beavatkozo_szerv_telites_hatasa/Scope2')
open_system('beavatkozo_szerv_telites_hatasa/Scope')

% Beavatkozo szerv telites kezelese
sim('beavatkozo_szerv_telites_kezelese');
open_system('beavatkozo_szerv_telites_kezelese/Scope3')
open_system('beavatkozo_szerv_telites_kezelese/Scope')

% Mintaveteles FOXBORO PI szabalyozo
sim('mintaveteles_FOXBORO_PI_szabalyozo')
open_system('mintaveteles_FOXBORO_PI_szabalyozo/Scope3')
open_system('mintaveteles_FOXBORO_PI_szabalyozo/Scope')

% Parameterek kiirasa a Workspace-re
disp(' ');
disp(['z_d=',num2str(z_d)]);
disp(['K_C=',num2str(K_C)]);
disp(' ');

% C kod generalas
disp('A szabalyozast megvalosito C kod:');
if(ALAPJEL_MEREDEKSEG_GYORSITASNAL || ALAPJEL_MEREDEKSEG_LASSITASNAL || BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL || BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)
    disp(' ');
end
if(ALAPJEL_MEREDEKSEG_GYORSITASNAL || ALAPJEL_MEREDEKSEG_LASSITASNAL)
    disp('// Alapjel meredekseg korlatja [m/s^2]');
end
if(ALAPJEL_MEREDEKSEG_GYORSITASNAL)
    disp(['#define ALAPJEL_MEREDEKSEG_GYORSITASNAL ',num2str(ALAPJEL_MEREDEKSEG_GYORSITASNAL)]);
end
if(ALAPJEL_MEREDEKSEG_LASSITASNAL)
    disp(['#define ALAPJEL_MEREDEKSEG_LASSITASNAL ',num2str(ALAPJEL_MEREDEKSEG_LASSITASNAL)]);
end
if(BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL || BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)
    disp('// Beavatkozo jel meredekseg korlatja [100/s]');
end
if(BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL)
    disp(['#define BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL ',num2str(BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL)]);
end
if(BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)
    disp(['#define BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL ',num2str(BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)]);
end
disp(' ');
disp('// Belso valtozok');
%disp('// Inverz statikus nemlinearitas LUT letrehozasa');
%disp(['float inv_stat_nonlinearity[',num2str(size(inv_y_stat,1)),'] = {']);
%for i = 1:size(inv_y_stat,1)
%    disp([char(9),num2str(inv_y_stat(i)),',']);
%end
%disp('};');
disp('int32_t alapjel = mmpersec;');
disp('static int32_t elozo_alapjel = 0;');
disp('static float beavatkozo_jel = 0;');
disp('static float elozo_beavatkozo_jel = 0;');
disp('static float pozitiv_visszacsatolas = 0;');
disp('float FOXBORO_bemeno_jel = 0;');
disp(' ');
disp('// Szabalyozas');
if(ALAPJEL_MEREDEKSEG_GYORSITASNAL || ALAPJEL_MEREDEKSEG_LASSITASNAL)
    disp('// Alapjel meredekseg korlatozas');
end
if(ALAPJEL_MEREDEKSEG_GYORSITASNAL)
    disp('if(alapjel - elozo_alapjel > ALAPJEL_MEREDEKSEG_GYORSITASNAL)');
    disp([char(9),'alapjel = elozo_alapjel + ALAPJEL_MEREDEKSEG_GYORSITASNAL;']);
end
if(ALAPJEL_MEREDEKSEG_LASSITASNAL)
    disp('if(elozo_alapjel - alapjel > ALAPJEL_MEREDEKSEG_LASSITASNAL)');
    disp([char(9),'alapjel = elozo_alapjel - ALAPJEL_MEREDEKSEG_LASSITASNAL;']);
end
disp('// Szabalyozasi algoritmus');
disp(['pozitiv_visszacsatolas = ',num2str(z_d),'*pozitiv_visszacsatolas+',num2str(1-z_d),'*beavatkozo_jel;']);
disp(['FOXBORO_bemeno_jel = ',num2str(K_C*743/10000),'*(mmpersec-velocity)+pozitiv_visszacsatolas;']);
disp('// Beavatkozo szerv telites kezelese');
disp(['if(FOXBORO_bemeno_jel > ',num2str(u_sat),')']);
disp([sprintf('\t'),'beavatkozo_jel = ',num2str(u_sat),';']);
disp(['else if(FOXBORO_bemeno_jel < ',num2str(-u_sat),')']);
disp([sprintf('\t'),'beavatkozo_jel = ',num2str(-u_sat),';']);
disp('else');
disp([sprintf('\t'),'beavatkozo_jel = FOXBORO_bemeno_jel;']);
if(BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL || BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)
    disp('// Beavatkozo jel meredekseg korlatozas');
end
if(BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL)
    disp('if(beavatkozo_jel - elozo_beavatkozo_jel > BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL)');
    disp([char(9),'beavatkozo_jel = elozo_beavatkozo_jel + BEAVATKOZO_JEL_MEREDEKSEG_GYORSITASNAL;']);
end
if(BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)
    disp('if(elozo_beavatkozo_jel - beavatkozo_jel > BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL)');
    disp([char(9),'beavatkozo_jel = elozo_beavatkozo_jel - BEAVATKOZO_JEL_MEREDEKSEG_LASSITASNAL;']);
end
disp('// Inverz statikus nemlinearitas');
disp('if(beavatkozo_jel > 0)');
disp('{');
disp([char(9),'if((',num2str(inv_u_stat(1)),' < beavatkozo_jel) && (beavatkozo_jel <= ',num2str(inv_u_stat(2)),'))']);
disp([char(9),char(9),'beavatkozo_jel = ',num2str(inv_y_stat(1)),'+',num2str((inv_y_stat(2)-inv_y_stat(1))/(inv_u_stat(2)-inv_u_stat(1))),'*beavatkozo_jel;']);
for i = 2:size(inv_u_stat,1)-1
    disp([char(9),'else if((',num2str(inv_u_stat(i)),' < beavatkozo_jel) && (beavatkozo_jel <= ',num2str(inv_u_stat(i+1)),'))']);
    disp([char(9),char(9),'beavatkozo_jel = ',num2str(inv_y_stat(i)),'+',num2str((inv_y_stat(i+1)-inv_y_stat(i))/(inv_u_stat(i+1)-inv_u_stat(i))),'*(beavatkozo_jel-',num2str(inv_u_stat(i)),');']);
end
disp('}');
disp('// Beavatkozo jel kiadasa');
disp('motorpulsePWM = (uint32_t) (beavatkozo_jel+6932);');
disp('// Integratorok feltoltese');
disp('elozo_alapjel = alapjel;');
disp('elozo_beavatkozo_jel = beavatkozo_jel;');
disp(' ');

y_stat = y_stat*K;
%%