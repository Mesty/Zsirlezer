
% Allapotvisszacsatolas szimulacioja

% A valtozoneveket atirni nem szabad, a Simulink modell ezeket hasznalja

%% Parameterek (valtozoneveket atirni nem szabad, csak az ertekeket)

% Kormanyszervo parameterezese (jelenleg tanszeki modell alapjan)
T_H = 0.03; % Holtido [s]
T_M = 0.01; % Motor idoallandoja [s]
fi_m = 20; % Kormanyszervo mechanikai korlatja az egyeneshez kepest [fok]
m = 260; % Kormanyszervo szogsebesseg korlatja [fok/s]
s_s = 52/3; % Maximalis szaturacio nelkuli szogsebesseg szoghibaja [fok]

% Jarmu parameterezese
L = 0.22; % Tengelytavolsag [m]
d = 0.065; % Elso szenzorsor tavolsaga az elso tengelytol [m]
p_1s = 0.0917605; % Elso szenzorsor szelessegenek fele [m]
p_2s = 0.068075; % Hatso szenzorsor szelessegenek fele [m]
L_sensor = 0.13; % Ket szenzorsor tavolsaga egymastol [m]

% Szabalyozo parameterek
kszi = 0.9; % Csillapitasi tenyezo
% d_5s 5%-os beallasi ut
% d_5s = a*v+b, ahol v sebesseg, a[s] meredekseg, b[m] konstans
a = 0.9; % d_5s meredeksege [s]
b = 0.5; % d_5s 0 sebesseghez tartozo erteke [m]

% Szoftver (kvantalas) parameterei
T_p_a = 0.01; % Vonalpozicio alapjel mintavetelezesi ideje [s]
T_v = 0.01; % Sebessegmeres mintavetelezesi ideje [s]
T_delta = 0.01; % Vonalorientacio meres mintavetelezesi ideje [s]
T_p = 0.01; % Vonalpozicio meres mintavetelezesi ideje [s]
T_PWM = 0.01; % Beavatkozo jel mintavetelezesi ideje [s]
p_1TCRTa = 100; % Elso szenzorsor minimalis erteke
p_1TCRTb = 3200; % Elso szenzorsor maximalis erteke
p_2TCRTa = 100; % Hatso szenzorsor minimalis erteke
p_2TCRTb = 2400; % Hatso szenzorsor maximalis erteke
fi_PWMa = 5000; % Legkisebb szoghoz tartozo PWM pulse
fi_PWMb = 9000; % Legnagyobb szoghoz tartozo PWM pulse

% Kezdeti feltetelek
p_init = -0.1; % Kezdeti vonalpozicio [m]
d_init = 0; % Kezdeti vonalorientacio [fok]
fi_init = 0; % Kezdeti kormanyszog [fok]

% Szimulacio parameterei
T_sim = 3; % Szimulacios ido [s]
p_a = timeseries(zeros(1,T_sim/T_p_a)); % Pozicio alapjel-ido fuggveny
v = timeseries(5*ones(1,T_sim/T_v)); % Sebesseg-ido fuggveny

%% Innentol a programkodot modositani nem szabad
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Szimulacio

% Mertekegysegek konvertalasa (fok -> radian)
fi_m = deg2rad(fi_m);
m = deg2rad(m);
s_s = deg2rad(s_s);
d_init = deg2rad(d_init);
% Mertekegysegek konvertalasa (fok -> PWM leptek)
fi_init = pi*fi_init*(fi_PWMb-fi_PWMa)/(360*fi_m);

sim('Allapotvisszacsatolas');
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% Abrazolas
% - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
% C kod generalas
%%