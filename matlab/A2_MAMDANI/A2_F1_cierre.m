
%% AVANCE 2 - CIERRE DE FASE 1
% Consolida evidencias sin modificar el controlador v01.

clearvars; clc;

assert(isfile("AMR_Mamdani_A2_v01.fis"));

fis = readfis("AMR_Mamdani_A2_v01.fis");
assert(numel(fis.Inputs)==3);
assert(numel(fis.Outputs)==2);
assert(numel(fis.Rules)==27);

% Archivos obligatorios
archivos = {
    "A2_F1_membresias.m"
    "A2_F1_reglas_27.m"
    "A2_F1_auditoria.m"
    "A2_F1_superficies.m"
    "AMR_Mamdani_A2_v01.fis"
    "A2_F1_tabla_27_reglas.csv"
    "A2_F1_membresias_base.mat"
    "A2_F1_modelo_27_reglas.mat"
    "A2_F1_auditoria_v01.mat"
};

for k = 1:numel(archivos)
    assert(isfile(archivos{k}), ...
        "Falta archivo: %s",archivos{k});
end

% Comprobar evidencias
tabla = readtable("A2_F1_tabla_27_reglas.csv");
barrido = readtable( ...
    "evidencias/A2_F1_barrido_1331.csv");
criticos = readtable( ...
    "evidencias/A2_F1_casos_criticos.csv");

assert(height(tabla)==27);
assert(height(barrido)==1331);
assert(height(criticos)==10);

imagenes = dir(fullfile("evidencias","*.png"));
assert(numel(imagenes)>=9, ...
    "Faltan figuras de membresias o superficies.");

% Resumen técnico
resumen = [
"# Avance 2 - Fase 1: Diseno Mamdani"
""
"Fecha: 2026-11-06"
"Estado: validado numericamente en MATLAB Online."
""
"## Configuracion"
"- Tipo: Mamdani."
"- Entradas: df, di, dd (0 a 2 m)."
"- Salidas: v (0 a 0.20 m/s) y omega (-1.5 a 1.5 rad/s)."
"- Operadores: AND=min, OR=max, implicacion=min."
"- Agregacion=max; defuzzificacion=centroide."
"- Convencion REP-103: omega positivo gira a izquierda."
""
"## Resultados"
"- 27 de 27 combinaciones cubiertas."
"- 1331 entradas evaluadas sin NaN ni Inf."
"- Velocidad observada: 0.011816 a 0.161163 m/s."
"- Giro observado: aproximadamente +/-1.165516 rad/s."
"- Pruebas criticas y direcciones de giro satisfactorias."
"- Cuatro superficies generadas."
""
"## Limitaciones"
"- Tres lados cercanos producen v=0.011816 m/s."
"- El controlador difuso no garantiza parada absoluta."
"- La seguridad estricta debe implementarse en el supervisor."
"- La ausencia de colisiones requiere ensayos en Gazebo."
"- Concordancia MATLAB-Python: pendiente de Fase 2."
""
"## Version"
"- AMR_Mamdani_A2_v01.fis: prototipo validado de Fase 1."
"- No se han modificado los artefactos del Avance 1."
];

archivoResumen = "A2_F1_resumen_tecnico.md";

assert(~isfile(archivoResumen), ...
    "El resumen ya existe; no se sobrescribira.");

fid = fopen(archivoResumen,"w");
assert(fid>0);

for linea = resumen.'
    fprintf(fid,"%s\n",char(linea));
end
fclose(fid);

% Incorporar evidencias
archivos{end+1} = archivoResumen;
archivos{end+1} = ...
    "evidencias/A2_F1_barrido_1331.csv";
archivos{end+1} = ...
    "evidencias/A2_F1_casos_criticos.csv";

for k = 1:numel(imagenes)
    archivos{end+1} = fullfile( ...
        "evidencias",imagenes(k).name);
end

% Empaquetado
nombreZIP = "A2_F1_cierre_20261106.zip";

assert(~isfile(nombreZIP), ...
    "El ZIP ya existe; no se sobrescribira.");

zip(char(nombreZIP),cellstr(string(archivos)));

disp("=== CIERRE FASE 1 ===")
fprintf("[OK] Reglas: %d/27\n",numel(fis.Rules));
fprintf("[OK] Barrido: %d evaluaciones\n",height(barrido));
fprintf("[OK] Casos criticos: %d\n",height(criticos));
fprintf("[OK] Figuras PNG: %d\n",numel(imagenes));
disp("[OK] Resumen tecnico generado.");
disp("[OK] ZIP de evidencias generado.");
disp("=== FASE 1 PREPARADA PARA GITHUB ===");
