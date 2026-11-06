
%% AVANCE 2 - FASE 1.4
% Auditoria numerica del controlador Mamdani v01
% No modifica el archivo FIS original.

clearvars; clc; close all;

archivo = "AMR_Mamdani_A2_v01.fis";
assert(isfile(archivo),"No se encontro el archivo FIS.");

fis = readfis(archivo);
opts = evalfisOptions(NumSamplePoints=1001);

assert(numel(fis.Inputs)==3);
assert(numel(fis.Outputs)==2);
assert(numel(fis.Rules)==27);

%% 1. Cobertura estructural de 27 combinaciones

antecedentes = zeros(27,3);

for k = 1:27
    antecedentes(k,:) = fis.Rules(k).Antecedent;
end

assert(all(antecedentes>=1 & antecedentes<=3,"all"));
assert(size(unique(antecedentes,"rows"),1)==27);

[a,b,c] = ndgrid(1:3,1:3,1:3);
combinaciones = [a(:),b(:),c(:)];

assert(isequal( ...
    sortrows(antecedentes), ...
    sortrows(combinaciones)));

disp("[OK] Cobertura estructural completa: 27/27.");

%% 2. Exploracion numerica de 1331 entradas

niveles = 0:0.2:2;

[df,di,dd] = ndgrid(niveles,niveles,niveles);

X = [df(:),di(:),dd(:)];

Y = evalfis(fis,X,opts);

assert(size(Y,1)==1331);
assert(size(Y,2)==2);
assert(all(isfinite(Y),"all"));

v = Y(:,1);
omega = Y(:,2);

assert(all(v>=-1e-9 & v<=0.20+1e-9));
assert(all(abs(omega)<=1.5+1e-9));

fprintf("\n=== EXPLORACION NUMERICA ===\n");
fprintf("Casos evaluados: %d\n",size(X,1));
fprintf("Velocidad minima: %.6f m/s\n",min(v));
fprintf("Velocidad maxima: %.6f m/s\n",max(v));
fprintf("Giro minimo: %.6f rad/s\n",min(omega));
fprintf("Giro maximo: %.6f rad/s\n",max(omega));

disp("[OK] Sin NaN ni valores infinitos.");
disp("[OK] Salidas dentro de los limites definidos.");

%% 3. Pruebas de escenarios criticos

nombres = [
    "PasilloLibre"
    "ObstaculoIzquierdo"
    "ObstaculoDerecho"
    "FrenteIzquierdaLibre"
    "FrenteDerechaLibre"
    "TresLadosCerca"
    "FrenteCercaLadosMedios"
    "FrenteCercaLadosLejos"
    "DistanciaFrontalUmbral"
    "DistanciaLateralUmbral"
];

casos = [
    1.7 1.7 1.7
    1.7 0.2 1.7
    1.7 1.7 0.2
    0.2 1.7 0.2
    0.2 0.2 1.7
    0.2 0.2 0.2
    0.2 0.9 0.9
    0.2 1.7 1.7
    0.6 0.9 0.9
    1.7 0.35 1.7
];

Z = evalfis(fis,casos,opts);

tablaCriticos = table( ...
    nombres, ...
    casos(:,1),casos(:,2),casos(:,3), ...
    Z(:,1),Z(:,2), ...
    VariableNames=["Escenario","df","di","dd","v","omega"]);

disp("=== ESCENARIOS CRITICOS ===");
disp(tablaCriticos);

%% 4. Coherencia de giro

assert(Z(2,2)<0, ...
    "Obstaculo izquierdo: giro incorrecto.");

assert(Z(3,2)>0, ...
    "Obstaculo derecho: giro incorrecto.");

assert(Z(4,2)>0, ...
    "Frente bloqueado: giro izquierdo incorrecto.");

assert(Z(5,2)<0, ...
    "Frente bloqueado: giro derecho incorrecto.");

assert(abs(Z(1,2))<0.05, ...
    "Pasillo libre: giro innecesario.");

assert(Z(1,1)>Z(4,1), ...
    "No disminuye velocidad con frente bloqueado.");

disp("[OK] Direccion de giro y reduccion verificadas.");

%% 5. Diagnostico de parada estricta

if Z(6,1)>1e-6
    fprintf("\n[ADVERTENCIA] Tres lados cercanos: ");
    fprintf("v = %.6f m/s, no es parada absoluta.\n",Z(6,1));
    disp("Se requiere proteccion de parada en supervisor.");
else
    disp("[OK] Parada estricta cuando los tres lados estan cerca.");
end

%% 6. Exportar evidencias

if ~isfolder("evidencias")
    mkdir("evidencias");
end

writetable(tablaCriticos, ...
    fullfile("evidencias","A2_F1_casos_criticos.csv"));

tablaBarrido = table( ...
    X(:,1),X(:,2),X(:,3),v,omega, ...
    VariableNames=["df","di","dd","v","omega"]);

writetable(tablaBarrido, ...
    fullfile("evidencias","A2_F1_barrido_1331.csv"));

save("A2_F1_auditoria_v01.mat", ...
    "X","Y","casos","Z","tablaCriticos");

disp("[OK] Tabla de casos criticos exportada.");
disp("[OK] Barrido de 1331 entradas exportado.");
disp("[OK] Resultados MAT guardados.");
disp("=== AUDITORIA FASE 1.4 FINALIZADA ===");
