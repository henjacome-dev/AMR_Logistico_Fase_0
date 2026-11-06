
%% Avance 2 - Fase 1.3
% Base exhaustiva de 27 reglas Mamdani
% Estados: 1=Cerca, 2=Media, 3=Lejos
% omega positivo = izquierda; negativo = derecha

clearvars; clc; close all;

assert(isfile("A2_F1_membresias_base.mat"), ...
    "Falta el modelo de la Etapa 1.2.");

datos = load("A2_F1_membresias_base.mat","fis");
fis = datos.fis;

assert(numel(fis.Inputs)==3);
assert(numel(fis.Outputs)==2);
assert(isempty(fis.Rules));

%% Construccion automatica de las 27 combinaciones
% Columnas:
% df | di | dd | v | omega | peso | conexion
% conexion 1 = AND

reglas = zeros(27,7);
n = 0;

for frontal = 1:3
    for izquierda = 1:3
        for derecha = 1:3

            if frontal == 1
                % Frente cercano: priorizar maniobra
                vel = 1;  % Detenido

                if izquierda == 1 && derecha == 1
                    giro = 3;  % Recto: sin espacio
                elseif izquierda >= derecha
                    giro = 5;  % IzquierdaFuerte
                else
                    giro = 1;  % DerechaFuerte
                end

            elseif frontal == 2
                % Frente a distancia intermedia
                vel = 2;  % Lenta

                if izquierda > derecha
                    giro = 4;  % IzquierdaSuave
                elseif izquierda < derecha
                    giro = 2;  % DerechaSuave
                else
                    giro = 3;  % Recto
                end

            else
                % Frente despejado
                if izquierda == 1 || derecha == 1
                    vel = 2;  % Lenta
                else
                    vel = 3;  % Rapida
                end

                if izquierda > derecha
                    giro = 4;  % IzquierdaSuave
                elseif izquierda < derecha
                    giro = 2;  % DerechaSuave
                else
                    giro = 3;  % Recto
                end
            end

            n = n + 1;
            reglas(n,:) = [ ...
                frontal izquierda derecha vel giro 1 1];
        end
    end
end

%% Validacion estructural

assert(n == 27);
assert(size(reglas,2) == 7);

antecedentes = reglas(:,1:3);

assert(size(unique(antecedentes,"rows"),1) == 27, ...
    "Se detectaron reglas duplicadas.");

assert(all(reglas(:,4)>=1 & reglas(:,4)<=3));
assert(all(reglas(:,5)>=1 & reglas(:,5)<=5));
assert(all(reglas(:,6)==1));
assert(all(reglas(:,7)==1));

fis = addRule(fis,reglas);

assert(numel(fis.Rules)==27, ...
    "La cantidad de reglas no es 27.");

disp("[OK] 27 reglas incorporadas.");
disp("[OK] 27 antecedentes distintos.");
disp("[OK] Sin combinaciones omitidas.");

%% Tabla legible de reglas

estados = ["Cerca","Media","Lejos"];

nombresV = ["Detenido","Lenta","Rapida"];

nombresW = [ ...
    "DerechaFuerte", ...
    "DerechaSuave", ...
    "Recto", ...
    "IzquierdaSuave", ...
    "IzquierdaFuerte"];

tablaTexto = strings(27,5);

for k = 1:27
    tablaTexto(k,:) = [ ...
        estados(reglas(k,1)), ...
        estados(reglas(k,2)), ...
        estados(reglas(k,3)), ...
        nombresV(reglas(k,4)), ...
        nombresW(reglas(k,5))];
end

tabla = array2table(tablaTexto, ...
    VariableNames=["df","di","dd","v","omega"]);

disp("=== MATRIZ DE LAS 27 REGLAS ===");
disp(tabla);

writetable(tabla,"A2_F1_tabla_27_reglas.csv");

%% Evaluacion numerica de los 27 centros linguisticos

centros = [0.20,0.90,1.70];

muestras = centros(antecedentes);

opciones = evalfisOptions(NumSamplePoints=1001);

salidas = evalfis(fis,muestras,opciones);

assert(isequal(size(salidas),[27 2]));
assert(all(isfinite(salidas),"all"));

assert(all(salidas(:,1)>=0));
assert(all(salidas(:,1)<=0.20));

assert(all(salidas(:,2)>=-1.5));
assert(all(salidas(:,2)<=1.5));

%% Pruebas de comportamiento

libre = evalfis(fis,[1.7 1.7 1.7],opciones);

obstaculoFrontal = ...
    evalfis(fis,[0.2 1.7 0.2],opciones);

obstaculoIzquierda = ...
    evalfis(fis,[1.7 0.2 1.7],opciones);

obstaculoDerecha = ...
    evalfis(fis,[1.7 1.7 0.2],opciones);

encerrado = evalfis(fis,[0.2 0.2 0.2],opciones);

assert(libre(1)>obstaculoFrontal(1), ...
    "No reduce velocidad ante obstaculo frontal.");

assert(obstaculoFrontal(2)>0, ...
    "Giro frontal incorrecto.");

assert(obstaculoIzquierda(2)<0, ...
    "Debe girar a la derecha.");

assert(obstaculoDerecha(2)>0, ...
    "Debe girar a la izquierda.");

assert(encerrado(1)<0.035, ...
    "Velocidad excesiva en situacion encerrada.");

assert(abs(encerrado(2))<0.05, ...
    "Giro inesperado en situacion encerrada.");

fprintf("\n=== RESULTADOS REPRESENTATIVOS ===\n");
fprintf("Pasillo libre:       v=%.5f, w=%+.5f\n",libre);
fprintf("Obstaculo frontal:   v=%.5f, w=%+.5f\n",obstaculoFrontal);
fprintf("Obstaculo izquierdo: v=%.5f, w=%+.5f\n",obstaculoIzquierda);
fprintf("Obstaculo derecho:   v=%.5f, w=%+.5f\n",obstaculoDerecha);
fprintf("Tres lados cerca:    v=%.5f, w=%+.5f\n",encerrado);

disp("[OK] Pruebas numericas principales aprobadas.");

%% Exportar prototipo de controlador

writeFIS(fis,"AMR_Mamdani_A2_v01.fis");

fisRecargado = readfis("AMR_Mamdani_A2_v01.fis");

assert(numel(fisRecargado.Rules)==27);

save("A2_F1_modelo_27_reglas.mat", ...
    "fis","reglas","muestras","salidas");

disp("[OK] Archivo FIS exportado y recargado.");
disp("[OK] Matriz CSV guardada.");
disp("[OK] Modelo MAT guardado.");
disp("=== ETAPA 1.3 FINALIZADA ===");
