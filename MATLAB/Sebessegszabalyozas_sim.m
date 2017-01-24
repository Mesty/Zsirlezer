
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

%% Parameterek (valtozoneveket atirni nem szabad, csak az ertekeket)

Ts = 0.1; % Mintavetelezesi ido [s]
T_cl = 0.5; % Zart kor eloirt idoallandoja [s]

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

%Mintaveteles FOXBORO PI szabalyozo
sim('mintaveteles_FOXBORO_PI_szabalyozo')
open_system('mintaveteles_FOXBORO_PI_szabalyozo/Scope3')
open_system('mintaveteles_FOXBORO_PI_szabalyozo/Scope')

y_stat = y_stat*K;
%%