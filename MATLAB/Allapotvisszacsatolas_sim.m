
% Allapotvisszacsatolas szimulacioja

% A valtozoneveket atirni nem szabad, a Simulink modell ezeket hasznalja

%% Workspace kiuritese

clear T_H T_M fi_m m s_s
clear L D p_1s p_2s L_sensor
clear kszi a b
clear T_p_a T_v T_delta T_p T_PWM p_1TCRTa p_1TCRTb p_2TCRTa p_2TCRTb fi_PWMa fi_PWMb
clear p_0 delta_0 fi_0
clear T_sim p_a v
clear tout dout p delta fi p_meas delta_meas
clear scrsz defaultFigurePosition vonalorientacio str

%% Gyakran valtoztatott parameterek (valtozoneveket atirni nem szabad)

p_0 = 0.06; % Kezdeti vonalpozicio [m]
delta_0 = 14; % Kezdeti vonalorientacio [fok]
v = 1.75; % Sebesseg [m/s]
% d_5s 5%-os beallasi ut
% d_5s = a*v+b, ahol v sebesseg, a[s] meredekseg, b[m] konstans
a = 0.573; % d_5s meredeksege [s]
b = 0.004; % d_5s 0 sebesseghez tartozo erteke [m]
kszi = 0.9; % Csillapitasi tenyezo

%% Osszes parameter (valtozoneveket atirni nem szabad, csak az ertekeket)
%  A gyakran valtoztatottak itt is megvannak, csak ki vannak kommentezve

% Kormanyszervo parameterezese (jelenleg tanszeki modell alapjan)
T_H = 0.03; % Holtido [s]
T_M = 0.01; % Motor idoallandoja [s]
fi_m = 24.5; % Kormanyszervo mechanikai korlatja az egyeneshez kepest [fok]
m = 260; % Kormanyszervo szogsebesseg korlatja [fok/s]
s_s = 52/3; % Maximalis szaturacio nelkuli szogsebesseg szoghibaja [fok]

% Jarmu parameterezese
L = 0.22; % Tengelytavolsag [m]
d = 0.065; % Elso szenzorsor tavolsaga az elso tengelytol [m]
p_1s = 0.0917605; % Elso szenzorsor szelessegenek fele [m]
p_2s = 0.068075; % Hatso szenzorsor szelessegenek fele [m]
L_sensor = 0.13; % Ket szenzorsor tavolsaga egymastol [m]

% Szabalyozo parameterek
%kszi = kszi; % Csillapitasi tenyezo
% d_5s 5%-os beallasi ut
% d_5s = a*v+b, ahol v sebesseg, a[s] meredekseg, b[m] konstans
% A szabalyozasi kor stabil, ha k_p+k_delta negativ
% Ezt ugy lehet biztositani, hogy k_p es k_delta 
% k_p = -9*(L+d)/(kszi^2*d_5s^2) -> mindig negativ
% k_delta = (L+d)/(kszi^2*d_5s^2)*(9*(L+d)-6*kszi^2*d_5s)
% csak akkor negativ, ha 9*(L+d) < 6*kszi^2*d_5s => b > 3*(L+d)/kszi^2
%a = (3-3*(L+d)/kszi^2-eps)/5; % d_5s meredeksege [s]
%b = 3*(L+d)/kszi^2+eps; % d_5s 0 sebesseghez tartozo erteke [m]

% Szoftver (kvantalas) parameterei
T_p_a = 0.003; % Vonalpozicio alapjel mintavetelezesi ideje [s]
T_v = 0.01; % Sebessegmeres mintavetelezesi ideje [s]
T_delta = 0.003; % Vonalorientacio meres mintavetelezesi ideje [s]
T_p = 0.003; % Vonalpozicio meres mintavetelezesi ideje [s]
T_PWM = 0.003; % Beavatkozo jel mintavetelezesi ideje [s]
p_1TCRTa = 100; % Elso szenzorsor minimalis erteke
p_1TCRTb = 3200; % Elso szenzorsor maximalis erteke
p_2TCRTa = 100; % Hatso szenzorsor minimalis erteke
p_2TCRTb = 2400; % Hatso szenzorsor maximalis erteke
SERVO_JOBB = 4610; % Legkisebb szoghoz tartozo PWM pulse
SERVO_BAL = 7250; % Legnagyobb szoghoz tartozo PWM pulse

% Kezdeti feltetelek
%p_0 = p_0; % Kezdeti vonalpozicio [m]
%delta_0 = delta_0; % Kezdeti vonalorientacio [fok]
fi_0 = 0; % Kezdeti kormanyszog [fok]

% Szimulacio parameterei
T_sim = 3; % Szimulacios ido [s]
p_a = 0; % Pozicio alapjel [m]
%v = v; % Sebesseg [m/s]

%% Innentol a programkodot modositani nem szabad
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Szimulacio

% Mertekegysegek konvertalasa (fok -> radian)
fi_m = deg2rad(fi_m);
m = deg2rad(m);
s_s = deg2rad(s_s);
delta_0 = deg2rad(delta_0);
% Mertekegysegek konvertalasa (fok -> PWM leptek)
fi_0 = pi*fi_0*(SERVO_BAL-SERVO_JOBB)/(360*fi_m);
% Beallasi uthossz szamitasa
d_5s = a*v+b;
% Szabalyozo parameterek szamitasa
k_p = -9*(L+d)/(kszi^2*d_5s^2);
k_delta = (L+d)/(kszi^2*d_5s^2)*(9*(L+d)-6*kszi^2*d_5s);

sim('Allapotvisszacsatolas');

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Abrazolas

% Eredmenyek kiirasa
disp(' ');
disp(['v=',num2str(v),'m/s,']);
disp(['p_0=',num2str(p_0),'m,']);
disp(['delta_0=',num2str(rad2deg(delta_0)),'�,']);
disp(['d_5%=',num2str(d_5s),'m,']);
disp(['t_5%=',num2str(d_5s/v),'s,']);
disp(['kszi=',num2str(kszi),',']);
disp(['k_p=',num2str(k_p),',']);
disp(['k_delta=',num2str(k_delta),'']);
disp(' ');

% Tavolsagtengelyek letrehozasa
p_meas.dist = v*p_meas.time;
p.dist = v*p.time;
delta_meas.dist = v*delta_meas.time;
delta.dist = v*delta.time;
fi.dist = v*fi.time;

% Abrazolas
scrsz = get(groot,'ScreenSize'); % Kepernyomeret, hogy szepen meretezzen
defaultFigurePosition = get(groot,'DefaultFigurePosition');
figure('Position',[1 defaultFigurePosition(2) scrsz(3) scrsz(4)/3],'Name','Allapotvisszacsatolas','NumberTitle','off')
subplot(1,3,1);
plot(p_meas.dist,p_meas.signals.values,'c');
hold on;
plot(p.dist,p.signals.values);
legend('p (m�rt)','p (val�s)','Location','SouthEast');
title('Vonalpoz�ci� [m]');
xlabel('Megtett �t [m]');
ylabel('');
grid on;
subplot(1,3,2);
plot(delta_meas.dist,delta_meas.signals.values,'c');
hold on;
plot(delta.dist,delta.signals.values);
legend('\delta (m�rt)','\delta (val�s)');
title('Vonalorient�ci� [�]');
xlabel('Megtett �t [m]');
ylabel('');
grid on;
subplot(1,3,3);
plot(fi.dist,fi.signals.values);
title('Korm�nysz�g [�]');
xlabel('Megtett �t [m]');
ylabel('');
grid on;

% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% C kod generalas

% Az algoritmus altal hasznalt ertekeket es szamitasokat lasd
% a papiron illetve a Simulink modellben
disp('A szabalyozast megvalosito C kod:');
disp(' ');
disp('// atan LUT inicializalasa (beszorozva k_delta konstans reszevel)');
disp(['float atan_lut[',num2str(abs(p_1TCRTa-p_2TCRTb-p_1TCRTb+p_2TCRTa)),'] = {']);
str = sprintf('');
for vonalorientacio = min(p_1TCRTa-p_2TCRTb,p_1TCRTb-p_2TCRTa):max(p_1TCRTa-p_2TCRTb,p_1TCRTb-p_2TCRTa)-1
    str = strcat(str,[' ',num2str(3*(L+d)/kszi^2*L*(SERVO_BAL-SERVO_JOBB)*atan((2*p_1s*vonalorientacio/(p_1TCRTb-p_1TCRTa)-p_1s*(p_1TCRTb+p_1TCRTa)/(p_1TCRTb-p_1TCRTa)+p_2s*(p_2TCRTb+p_2TCRTa)/(p_2TCRTb-p_2TCRTa))/L_sensor)/(2*fi_m*(L+d))),',']);
    if mod(vonalorientacio,10) == 0
        disp([char(9),str]);
        clear str;
        str = sprintf('');
    end
end
disp([char(9),str,' ',num2str(3*(L+d)/kszi^2*L*(SERVO_BAL-SERVO_JOBB)*atan((2*p_1s*max(p_1TCRTa-p_2TCRTb,p_1TCRTb-p_2TCRTa)/(p_1TCRTb-p_1TCRTa)-p_1s*(p_1TCRTb+p_1TCRTa)/(p_1TCRTb-p_1TCRTa)+p_2s*(p_2TCRTb+p_2TCRTa)/(p_2TCRTb-p_2TCRTa))/L_sensor)/(2*fi_m*(L+d))),'};']);
disp(' ');
disp('// A szabalyozas megvalositasa');
disp(['*PWMeredmeny = (uint32_t) (',num2str(-4.5*L*(SERVO_BAL-SERVO_JOBB)*p_1s/kszi^2),'*(2*((float)*pozicio)-',num2str(p_1TCRTb+p_1TCRTa),')/(',num2str((fi_m*(p_1TCRTb-p_1TCRTa))),'*(',num2str(a),'*sebesseg_a_szabalyozonak+',num2str(b),')*(',num2str(a),'*sebesseg_a_szabalyozonak+',num2str(b),'))+(',num2str(3*(L+d)-2*kszi^2*b),num2str(-2*kszi^2*a),'*sebesseg_a_szabalyozonak)*atan_lut[*orientacio+',num2str(-min(p_1TCRTa-p_2TCRTb,p_1TCRTb-p_2TCRTa)),']/((',num2str(a),'*sebesseg_a_szabalyozonak+',num2str(b),')*(',num2str(a),'*sebesseg_a_szabalyozonak+',num2str(b),'))+',num2str((SERVO_JOBB+SERVO_BAL)/2),');']);
disp(' ');
%%