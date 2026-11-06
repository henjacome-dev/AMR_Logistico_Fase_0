%% AVANCE 2 - FASE 1 - CIERRE ROBUSTO
% Consolida evidencias en ZIP sin alterar el FIS, los MAT o los CSV.
% Version: 2026-11-06. Ejecutar desde matlab/A2_MAMDANI.
% Prerequisito: auditoria complementaria aprobada en MATLAB Online.

clearvars; clc;

%% 1. Archivos obligatorios (incluye este script y la auditoria)
obligatorios = [
    "A2_F1_membresias.m"
    "A2_F1_reglas_27.m"
    "A2_F1_auditoria.m"
    "A2_F1_superficies.m"
    "A2_F1_cierre.m"
    "A2_F1_reparar_png.m"
    "A2_F1_verificacion_auditoria.m"
    "AMR_Mamdani_A2_v01.fis"
    "A2_F1_tabla_27_reglas.csv"
    "A2_F1_membresias_base.mat"
    "A2_F1_modelo_27_reglas.mat"
    "A2_F1_auditoria_v01.mat"
    "A2_F1_resumen_tecnico.md"
    "evidencias/A2_F1_barrido_1331.csv"
    "evidencias/A2_F1_casos_criticos.csv"
];

for k = 1:numel(obligatorios)
    assert(isfile(obligatorios(k)), ...
        "Falta archivo obligatorio: %s", obligatorios(k));
end

%% 2. Validacion estructural y tabular
fis = readfis("AMR_Mamdani_A2_v01.fis");
assert(numel(fis.Inputs)==3 && numel(fis.Outputs)==2);
assert(numel(fis.Rules)==27);
antecedentes = zeros(27,3);
for k=1:27
    antecedentes(k,:) = fis.Rules(k).Antecedent;
end
assert(size(unique(antecedentes,'rows'),1)==27, ...
    "Hay antecedentes duplicados en la base de reglas.");

reglas = readtable("A2_F1_tabla_27_reglas.csv");
barrido = readtable("evidencias/A2_F1_barrido_1331.csv");
criticos = readtable("evidencias/A2_F1_casos_criticos.csv");
assert(height(reglas)==27);
assert(height(barrido)==1331);
assert(height(criticos)==10);

%% 3. Nueve evidencias exactas: existen, tienen datos y se decodifican
nombresPNG = [
    "entrada_df.png"
    "entrada_di.png"
    "entrada_dd.png"
    "salida_v.png"
    "salida_omega.png"
    "superficie_v_df_di_dd_lejos.png"
    "superficie_omega_di_dd_df_cerca.png"
    "superficie_omega_di_dd_df_lejos.png"
    "superficie_v_df_dd_di_media.png"
];

imagenes = dir(fullfile("evidencias","*.png"));
assert(numel(imagenes)==numel(nombresPNG), ...
    "Se esperaban exactamente nueve archivos PNG.");

rutasPNG = strings(numel(nombresPNG),1);
for k=1:numel(nombresPNG)
    ruta = fullfile("evidencias",nombresPNG(k));
    assert(isfile(ruta),"Falta evidencia: %s",ruta);
    meta = dir(ruta);
    assert(meta.bytes>100,"Archivo PNG sin contenido suficiente: %s",ruta);
    datos = imfinfo(ruta);
    imagen = imread(ruta);
    assert(datos.Width>0 && datos.Height>0 && ~isempty(imagen), ...
        "No se pudo validar la imagen: %s",ruta);
    rutasPNG(k)=ruta;
end

disp('[OK] Nueve evidencias PNG verificadas con imfinfo e imread.');

%% 4. Empaquetado versionado, sin sobrescritura del original
archivos = [obligatorios; rutasPNG];
assert(numel(archivos)==24);

fecha = string(datetime('now','Format','yyyyMMdd_HHmmss'));
nombreZIP = "A2_F1_cierre_validado_"+fecha+".zip";
assert(~isfile(nombreZIP),"Ya existe este ZIP: %s",nombreZIP);

zip(char(nombreZIP), cellstr(archivos));
assert(isfile(nombreZIP));

%% 5. Prueba de extraccion y lectura del contenido del ZIP
prueba = tempname;
mkdir(prueba);
limpieza = onCleanup(@() rmdir(prueba,'s'));
unzip(char(nombreZIP),prueba);
for k=1:numel(archivos)
    assert(isfile(fullfile(prueba,archivos(k))), ...
        "El ZIP no contiene %s",archivos(k));
end
for k=1:numel(rutasPNG)
    ruta = fullfile(prueba,rutasPNG(k));
    imfinfo(ruta);
    imagen = imread(ruta);
    assert(~isempty(imagen));
end
fisCopia=readfis(fullfile(prueba,"AMR_Mamdani_A2_v01.fis"));
assert(numel(fisCopia.Rules)==27);

%% 6. Resultado
datosZIP=dir(nombreZIP);
disp('=== CIERRE FASE 1 - INTEGRIDAD VERIFICADA ===');
fprintf('[OK] FIS: %d entradas, %d salidas, %d reglas.\n', ...
    numel(fis.Inputs),numel(fis.Outputs),numel(fis.Rules));
fprintf('[OK] CSV: %d barrido y %d criticos.\n',height(barrido),height(criticos));
fprintf('[OK] PNG legibles: %d/9.\n',numel(nombresPNG));
fprintf('[OK] ZIP: %s (%d bytes, %d archivos).\n', ...
    nombreZIP,datosZIP.bytes,numel(archivos));
disp('[ADVERTENCIA] Seguridad ROS 2 e integracion no certificadas.');
