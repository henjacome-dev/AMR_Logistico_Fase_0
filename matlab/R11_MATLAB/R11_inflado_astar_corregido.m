%% R11 - Inflado y planificacion A* con el escenario corregido
% Avance 1 - AMR logistico (MATLAB Online / Robotics System Toolbox)
% No modifica el paquete historico matlab/F124_MATLAB.
clearvars; clc; close all;

base = fileparts(mfilename('fullpath'));
destino = fullfile(base, 'resultados');
if ~isfolder(destino)
    mkdir(destino);
end

archivoBase = fullfile(base, 'almacen_f1.pgm');
archivoPython = fullfile(base, 'almacen_f1_inflado_python.pgm');
assert(isfile(archivoBase), 'Falta almacen_f1.pgm (mapa base corregido).');
assert(isfile(archivoPython), 'Falta almacen_f1_inflado_python.pgm.');

imagenBase = imread(archivoBase);
imagenPython = imread(archivoPython);
assert(isequal(size(imagenBase), [160 200]), 'Dimensiones inesperadas del mapa base.');
assert(isequal(size(imagenPython), size(imagenBase)), 'Los mapas difieren en dimensiones.');

resolucion = 20;       % celdas por metro = 0.05 m/celda
radio = 0.21;          % metros
origen = [-5, -4];    % esquina inferior izquierda, marco world

ocupacionBase = imagenBase < 128;
ocupacionPython = imagenPython < 128;
mapa = binaryOccupancyMap(ocupacionBase, resolucion);
mapa.GridLocationInWorld = origen;

% Comprobar que MATLAB lee correctamente la misma matriz base.
assert(isequal(logical(getOccupancy(mapa)), ocupacionBase), ...
    'La matriz de ocupacion base no coincide tras importarse a MATLAB.');

% Funcion exigida en la tarea 1.5: inflado en MATLAB (no importado desde Python).
inflate(mapa, radio);
ocupacionMATLAB = logical(getOccupancy(mapa));

nPython = nnz(ocupacionPython);
nMATLAB = nnz(ocupacionMATLAB);
soloMATLAB = nnz(ocupacionMATLAB & ~ocupacionPython);
soloPython = nnz(ocupacionPython & ~ocupacionMATLAB);
diferentes = nnz(xor(ocupacionMATLAB, ocupacionPython));

fprintf('=== R11: COMPARACION DE INFLADO ===\n');
fprintf('Resolucion: %.3f m/celda; radio solicitado: %.3f m\n', 1/resolucion, radio);
fprintf('Ocupadas Python: %d | MATLAB: %d\n', nPython, nMATLAB);
fprintf('Solo MATLAB: %d | Solo Python: %d | Diferentes: %d\n', ...
    soloMATLAB, soloPython, diferentes);
fprintf('NOTA: MATLAB puede redondear el radio a celdas; diferencias no implican error.\n');

metricas = table(nPython, nMATLAB, soloMATLAB, soloPython, diferentes, ...
    'VariableNames', {'ocupadas_python','ocupadas_matlab', ...
    'solo_matlab','solo_python','celdas_diferentes'});
writetable(metricas, fullfile(destino, 'R11_comparacion_inflado.csv'));

imwrite(uint8(~ocupacionMATLAB) * 254, ...
    fullfile(destino, 'R11_inflado_matlab.png'));
fig = figure('Visible', 'off', 'Name', 'R11 - Inflado MATLAB');
show(mapa, 'world');
title('R11 - Mapa corregido inflado en MATLAB (r = 0.21 m)');
exportgraphics(fig, fullfile(destino, 'R11_mapa_world.png'), 'Resolution', 180);
close(fig);

%% Rutas A* en el mapa inflado de MATLAB
puntos = [
    -3.0 -2.8;   % ROBOT
    -4.0 -2.8;   % RECEPCION
    -4.0  2.8;   % ALMACENAMIENTO
     4.0  2.8;   % EMPAQUE
     4.0 -2.8    % DESPACHO
];
nombres = ["ROBOT"; "RECEPCION"; "ALMACENAMIENTO"; "EMPAQUE"; "DESPACHO"];
planificador = plannerAStarGrid(mapa, 'DiagonalSearch', 'on');

origenRuta = nombres(1:4);
destinoRuta = nombres(2:5);
longitudRuta = zeros(4,1);
nodosRuta = zeros(4,1);
tiempoRuta_ms = zeros(4,1);
for k = 1:4
    tic;
    ruta = plan(planificador, puntos(k,:), puntos(k+1,:), 'world');
    tiempoRuta_ms(k) = toc * 1000;
    assert(~isempty(ruta), 'Ruta individual %d no encontrada.', k);
    longitudRuta(k) = sum(hypot(diff(ruta(:,1)), diff(ruta(:,2))));
    nodosRuta(k) = size(ruta,1);
    writetable(array2table(ruta, 'VariableNames', {'x_m','y_m'}), ...
        fullfile(destino, sprintf('R11_ruta_%d.csv', k)));
end

resumenRutas = table(origenRuta, destinoRuta, longitudRuta, nodosRuta, tiempoRuta_ms, ...
    'VariableNames', {'origen','destino','longitud_m','nodos','tiempo_ms'});
writetable(resumenRutas, fullfile(destino, 'R11_resumen_rutas.csv'));

% Las 3 ordenes ya establecidas por el proyecto.
ordenes = [1 2 3 4 5; 1 2 4 3 5; 1 5 4 3 2];
ids = ["A"; "B"; "C"];
longitudOrden = zeros(3,1);
nodosOrden = zeros(3,1);
tiempoOrden_ms = zeros(3,1);
for k = 1:3
    orden = ordenes(k,:);
    trayectoria = zeros(0,2);
    for j = 1:4
        tic;
        segmento = plan(planificador, puntos(orden(j),:), ...
            puntos(orden(j+1),:), 'world');
        tiempoOrden_ms(k) = tiempoOrden_ms(k) + toc * 1000;
        assert(~isempty(segmento), 'Orden %s tramo %d no encontrado.', char(ids(k)), j);
        if j > 1
            segmento = segmento(2:end,:);
        end
        trayectoria = [trayectoria; segmento]; %#ok<AGROW>
    end
    longitudOrden(k) = sum(hypot(diff(trayectoria(:,1)), diff(trayectoria(:,2))));
    nodosOrden(k) = size(trayectoria,1);
    writetable(array2table(trayectoria, 'VariableNames', {'x_m','y_m'}), ...
        fullfile(destino, sprintf('R11_orden_%s.csv', char(ids(k)))));
end
resumenOrdenes = table(ids, longitudOrden, nodosOrden, tiempoOrden_ms, ...
    'VariableNames', {'orden','longitud_m','nodos','tiempo_ms'});
writetable(resumenOrdenes, fullfile(destino, 'R11_resumen_ordenes.csv'));

fprintf('\n=== R11: RUTAS MATLAB SOBRE MAPA CORREGIDO ===\n');
disp(resumenRutas);
disp(resumenOrdenes);
[~, idx] = min(longitudOrden);
fprintf('[OK] 4 rutas, 3 ordenes; menor longitud: orden %s\n', char(ids(idx)));
fprintf('[OK] Evidencias generadas en: %s\n', destino);
