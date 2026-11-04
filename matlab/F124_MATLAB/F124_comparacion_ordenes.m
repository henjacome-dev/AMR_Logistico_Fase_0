
%% F1.2.4 - Comparacion de tres ordenes logisticas

clearvars; clc; close all;

base = fileparts(mfilename('fullpath'));

carpetaResultados = fullfile(base, 'resultados');
if ~isfolder(carpetaResultados)
    mkdir(carpetaResultados);
end

%% 1. Cargar mapa

archivoMapa = fullfile(base, 'mapas', ...
    'almacen_f1_inflado.pgm');

assert(isfile(archivoMapa), ...
    'Falta el mapa: %s', archivoMapa);

imagen = imread(archivoMapa);

if ndims(imagen) == 3
    imagen = imagen(:,:,1);
end

mapa = binaryOccupancyMap(imagen < 128, 20);
mapa.GridLocationInWorld = [-5 -4];

planificador = plannerAStarGrid( ...
    mapa, 'DiagonalSearch', 'on');

%% 2. Coordenadas

puntos = [
    -3.0 -2.8;  % Robot
    -4.0 -2.8;  % Recepcion
    -4.0  2.8;  % Almacenamiento
     4.0  2.8;  % Empaque
     4.0 -2.8   % Despacho
];

nombres = [
    "ROBOT"
    "RECEPCION"
    "ALMACENAMIENTO"
    "EMPAQUE"
    "DESPACHO"
];

%% 3. Definir ordenes logisticas

ordenes = [
    1 2 3 4 5;  % Orden A
    1 2 4 3 5;  % Orden B
    1 5 4 3 2   % Orden C
];

ids = ["A"; "B"; "C"];

longitudes = zeros(3,1);
tiempos = zeros(3,1);
celdas = zeros(3,1);
secuencias = strings(3,1);

%% 4. Planificar las tres ordenes

for k = 1:3

    orden = ordenes(k,:);
    trayectoria = zeros(0,2);
    tiempo_total = 0;

    for j = 1:4

        inicio = puntos(orden(j),:);
        meta = puntos(orden(j+1),:);

        tic;

        segmento = plan( ...
            planificador, inicio, meta, 'world');

        tiempo_total = tiempo_total + toc*1000;

        assert(~isempty(segmento), ...
            'No existe ruta en orden %s, tramo %d', ...
            char(ids(k)), j);

        % Eliminar punto duplicado entre tramos
        if j > 1
            segmento = segmento(2:end,:);
        end

        trayectoria = [trayectoria; segmento];

    end

    %% 5. Calcular metricas

    longitudes(k) = sum(hypot( ...
        diff(trayectoria(:,1)), ...
        diff(trayectoria(:,2))));

    tiempos(k) = tiempo_total;
    celdas(k) = size(trayectoria,1);

    secuencias(k) = string(strjoin( ...
        cellstr(nombres(orden(2:end))), '-'));

    %% 6. Exportar trayectoria

    tablaRuta = array2table(trayectoria, ...
        'VariableNames', {'x_m','y_m'});

    writetable(tablaRuta, ...
        fullfile(carpetaResultados, ...
        sprintf('F124_orden_%s.csv', char(ids(k)))));

    fprintf( ...
        'Orden %s: %.3f m | %.2f ms | %d celdas\n', ...
        char(ids(k)), longitudes(k), ...
        tiempos(k), celdas(k));

end

%% 7. Tabla resumen MATLAB

resumen = table( ...
    ids(:), ...
    secuencias(:), ...
    celdas(:), ...
    longitudes(:), ...
    tiempos(:));

resumen.Properties.VariableNames = { ...
    'orden', 'secuencia', 'celdas', ...
    'longitud_m', 'tiempo_ms'};

disp('=== RESULTADOS MATLAB ===');
disp(resumen);

writetable(resumen, ...
    fullfile(carpetaResultados, ...
    'F124_comparacion_ordenes.csv'));

%% 8. Comparacion con Python

archivoPython = fullfile( ...
    carpetaResultados, ...
    'F12_comparacion_ordenes.csv');

if isfile(archivoPython)

    python = readtable(archivoPython, ...
        'TextType', 'string');

    assert(all(ismember( ...
        {'orden','longitud_m'}, ...
        python.Properties.VariableNames)), ...
        'CSV Python requiere orden y longitud_m');

    [encontradas, indices] = ismember( ...
        ids(:), strtrim(string(python.orden)));

    assert(all(encontradas), ...
        'Faltan las ordenes A, B o C en Python');

    python_m = python.longitud_m(indices);
    matlab_m = longitudes(:);

    diferencias = matlab_m - python_m(:);

    comparacion = table( ...
        ids(:), ...
        python_m(:), ...
        matlab_m(:), ...
        diferencias(:));

    comparacion.Properties.VariableNames = { ...
        'orden', 'python_m', 'matlab_m', ...
        'diferencia_m'};

    disp('=== COMPARACION PYTHON VS MATLAB ===');
    disp(comparacion);

    fprintf('Diferencia absoluta maxima: %.3f m\n', ...
        max(abs(diferencias)));

    writetable(comparacion, ...
        fullfile(carpetaResultados, ...
        'F124_comparacion_final.csv'));

else

    fprintf(['No se encontro CSV de Python. ', ...
        'Resultados MATLAB guardados.\n']);

end

%% 9. Determinar mejor orden

[~, mejor] = min(longitudes);

fprintf('\nMEJOR ORDEN MATLAB: %s\n', ...
    char(ids(mejor)));

disp('F1.2.4: COMPARACION COMPLETADA');
