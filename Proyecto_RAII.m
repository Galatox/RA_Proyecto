clear all; close all; clc;
% Para trabajar en CCR
A = 511; % Valor máximo de entrada CCR
K = 204/A; % Ganancia de la planta
T = 0.06; % Constante T del sistema de primer orden
Ts = 1/100; % Tiempo de muestreo
% Modelado de la planta
s = tf('s');
z = tf('z');
G = K/(T*s+1); % Modelo de la planta en continuo
Gz = c2d(G,Ts,'zoh'); % Modelo de la platna en discreto

Rzvel = zpk(1.9624*tf([1 -0.8474],[1 -1],Ts));
Rsvel = zpk(d2c(Rzvel));


Gvelf = feedback(Rsvel * G,1);

Gpos = zpk(Gvelf * 1/s);
Gpoz = zpk(c2d(Gpos,Ts,'zoh'))

Gposdeg = zpk(Gvelf * 1/s);
Gpozdeg = zpk(c2d(Gpos,Ts,'zoh'))


