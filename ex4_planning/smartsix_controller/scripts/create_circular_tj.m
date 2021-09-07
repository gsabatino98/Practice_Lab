clear all;
close all;
clc;


filepath = '~/roblab_ws/src/ex4_planning/smartsix_controller/data/circular.traj';

%% indico il numero di punti su cui tj deve interpolare
no_of_cp = 10;

%% indico il campionamento della traiettoria (il campionamento deve essere 
%  più fitto poiché ho una funzione continua).
no_of_samples = 25;

%% definisco una circonferanza in coordiate polari che vanno da pi a -pi (quindi tutti i theta)
%  definisco un numero di theta introno alla circonferenza di 360° pari a
%  no_of_cp
rho = 0.4;
theta = linspace(-pi, pi, no_of_cp);

%% inputs
%  definisco il vettore contenente x,y,z (vettori di dimensione 10)
%  in questa configurazione ottengo che la mia flangia punta di fronte al
%  manipolatore
ctrl_points = NaN * ones(3, no_of_cp);
%  x
ctrl_points(1,:) = 0.45 * ones(1, no_of_cp);
%  y
ctrl_points(2,:) = (rho * sin(theta));
%  z
ctrl_points(3,:) = 1.15+(rho * cos(theta));

%% funzione per l'interpolazione (di tipo spline)
%  in questa funzione si definiscono rispettivamente i punti in ingresso, i
%  punti in uscita ed se si vuole visualizzare o meno la traiettoria
%  ottenuta dall' interpolazione
[x_lambda, lambda] = generate_path(ctrl_points, no_of_samples, true);

x_lambda(4:6,:) = zeros(3, no_of_samples);

export_ros_workspace_path(filepath, lambda, x_lambda);