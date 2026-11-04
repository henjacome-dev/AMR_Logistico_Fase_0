
%% F1.2.4 - Validacion de cuatro rutas A* en MATLAB Online

clearvars; clc; close all;

base = fileparts(mfilename('fullpath'));

carpetaResultados = fullfile(base, 'resultados');
if ~isfolder(carpetaResultados)
    mkdir(carpetaResultados);
end

%% 1. Cargar mapa inflado

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

%% 2. Posiciones del robot y estaciones

nombres = [
    "ROBOT"
    "RECEPCION"
    "ALMACENAMIENTO"
    "EMPAQUE"
    "DESPACHO"
];

puntos = [
    -3.0 -2.8;
    -4.0 -2.8;
    -4.0  2.8;
     4.0  2.8;
     4.0 -2.8
];

origen = nombres(1:4);
destino = nombres(2:5);

longitud_m = zeros(4,1);
tiempo_ms = zeros(4,1);
nodos = zeros(4,1);
explorados = zeros(4,1);

%% 3. Crear grafica

fig = figure('Name', ...
    'F1.2.4 - Cuatro rutas A*');

show(mapa, 'world');
hold on;

colores = lines(4);
h = gobjects(4,1);

%% 4. Calcular las cuatro rutas

for k = 1:4

    inicio = puntos(k,:);
    meta = puntos(k+1,:);

    tic;

    [ruta, informacion] = plan( ...
        planificador, inicio, meta, 'world');

    tiempo_ms(k) = toc * 1000;

    assert(~isempty(ruta), ...
        'No se encontro la ruta %d', k);

    longitud_m(k) = sum(hypot( ...
        diff(ruta(:,1)), ...
        diff(ruta(:,2))));

    nodos(k) = size(ruta,1);
    explorados(k) = informacion.NumNodesExplored;

    % Exportar CSV de cada ruta
    tablaRuta = array2table(ruta, ...
        'VariableNames', {'x_m','y_m'});

    writetable(tablaRuta, ...
        fullfile(carpetaResultados, ...
        sprintf('F124_ruta_%d.csv', k)));

    % Dibujar ruta
    h(k) = plot( ...
        ruta(:,1), ruta(:,2), '-', ...
        'Color', colores(k,:), ...
        'LineWidth', 2);

    fprintf( ...
        'Ruta %d: %.3f m | %.3f ms | %d nodos\n', ...
        k, longitud_m(k), tiempo_ms(k), nodos(k));

end

%% 5. Finalizar grafica

plot(puntos(:,1), puntos(:,2), 'ro', ...
    'MarkerSize', 6, ...
    'MarkerFaceColor', 'r');

axis equal;
xlabel('X (m)');
ylabel('Y (m)');

title('F1.2.4 - Cuatro rutas A* en MATLAB');

legend(h, { ...
    'Robot - Recepcion', ...
    'Recepcion - Almacenamiento', ...
    'Almacenamiento - Empaque', ...
    'Empaque - Despacho'}, ...
    'Location', 'bestoutside');

hold off;

exportgraphics(fig, ...
    fullfile(carpetaResultados, ...
    'F124_rutas_astar.png'), ...
    'Resolution', 180);

%% 6. Tabla de resultados MATLAB

resumen = table( ...
    origen(:), ...
    destino(:), ...
    longitud_m(:), ...
    tiempo_ms(:), ...
    nodos(:), ...
    explorados(:));

resumen.Properties.VariableNames = { ...
    'origen', 'destino', 'longitud_m', ...
    'tiempo_ms', 'nodos', 'explorados'};

disp('=== RESULTADOS MATLAB ===');
disp(resumen);

writetable(resumen, ...
    fullfile(carpetaResultados, ...
    'F124_resumen_rutas.csv'));

%% 7. Comparacion opcional con Python

archivoPython = fullfile( ...
    carpetaResultados, 'F12_resumen_rutas.csv');

if isfile(archivoPython)

    python = readtable(archivoPython, ...
        'TextType', 'string');

    columnas = python.Properties.VariableNames;

    if all(ismember( ...
            {'origen','destino','longitud_m'}, columnas))

        python_m = nan(4,1);

        for k = 1:4

            indice = find( ...
                strcmpi( ...
                    strtrim(string(python.origen)), ...
                    origen(k)) & ...
                strcmpi( ...
                    strtrim(string(python.destino)), ...
                    destino(k)), 1);

            assert(~isempty(indice), ...
                'Falta una ruta en el CSV Python');

            python_m(k) = python.longitud_m(indice);

        end

        diferencia_m = longitud_m(:) - python_m;

        comparacion = table( ...
            origen(:), destino(:), python_m, ...
            longitud_m(:), diferencia_m);

        comparacion.Properties.VariableNames = { ...
            'origen','destino','python_m', ...
            'matlab_m','diferencia_m'};

        disp('=== DIFERENCIAS MATLAB - PYTHON ===');
        disp(comparacion);

        fprintf('Diferencia absoluta maxima: %.3f m\n', ...
            max(abs(diferencia_m)));

        writetable(comparacion, ...
            fullfile(carpetaResultados, ...
            'F124_comparacion_rutas.csv'));

    else

        warning(['CSV Python sin las columnas ', ...
            'necesarias. Se omite la comparacion.']);

    end

else

    fprintf(['No se encontro F12_resumen_rutas.csv. ', ...
        'Resultados MATLAB guardados.\n']);

end

disp('F1.2.4: VALIDACION DE CUATRO RUTAS FINALIZADA');
