%% AVANCE 2 - AUDITORIA COMPLEMENTARIA DE FASE 1
% Comprobacion de lectura y reproducibilidad en MATLAB Online.
% SOLO LECTURA: no genera, sobrescribe ni elimina ningun archivo.
% Ejecutar dentro de matlab/A2_MAMDANI.

clearvars; clc;
disp('=== AUDITORIA COMPLEMENTARIA FASE 1 ===');

%% 1. Archivos obligatorios
obligatorios = [
    "AMR_Mamdani_A2_v01.fis"
    "A2_F1_membresias_base.mat"
    "A2_F1_modelo_27_reglas.mat"
    "A2_F1_auditoria_v01.mat"
    "A2_F1_tabla_27_reglas.csv"
    "A2_F1_resumen_tecnico.md"
    "evidencias/A2_F1_barrido_1331.csv"
    "evidencias/A2_F1_casos_criticos.csv"
];
for k = 1:numel(obligatorios)
    assert(isfile(obligatorios(k)), ...
        "Falta archivo obligatorio: %s",obligatorios(k));
end
disp('[OK] Archivos tecnicos basicos presentes.');

%% 2. Carga REAL de archivos MAT y objetos MATLAB
M0 = load('A2_F1_membresias_base.mat');
M1 = load('A2_F1_modelo_27_reglas.mat');
M2 = load('A2_F1_auditoria_v01.mat');

assert(isfield(M0,'fis') && isa(M0.fis,'mamfis'));
assert(isfield(M1,'fis') && isa(M1.fis,'mamfis'));
assert(isfield(M1,'reglas') && isequal(size(M1.reglas),[27 7]));
assert(isfield(M1,'muestras') && isequal(size(M1.muestras),[27 3]));
assert(isfield(M1,'salidas') && isequal(size(M1.salidas),[27 2]));
assert(isempty(M0.fis.Rules));
assert(numel(M1.fis.Rules)==27);

assert(isfield(M2,'X') && isequal(size(M2.X),[1331 3]));
assert(isfield(M2,'Y') && isequal(size(M2.Y),[1331 2]));
assert(isfield(M2,'casos') && isequal(size(M2.casos),[10 3]));
assert(isfield(M2,'Z') && isequal(size(M2.Z),[10 2]));
assert(isfield(M2,'tablaCriticos') && istable(M2.tablaCriticos));
assert(height(M2.tablaCriticos)==10);
assert(all(isfinite(M2.X),'all') && all(isfinite(M2.Y),'all'));
assert(all(isfinite(M2.casos),'all') && all(isfinite(M2.Z),'all'));
disp('[OK] Tres MAT cargados y variables internas comprobadas.');

%% 3. Verificacion del FIS sin modificarlo
F = readfis('AMR_Mamdani_A2_v01.fis');
assert(numel(F.Inputs)==3 && numel(F.Outputs)==2);
assert(numel(F.Rules)==27);
assert(isequal([F.Inputs(1).Range; F.Inputs(2).Range; F.Inputs(3).Range], ...
    [0 2; 0 2; 0 2]));
assert(isequal(F.Outputs(1).Range,[0 0.2]));
assert(isequal(F.Outputs(2).Range,[-1.5 1.5]));

ant = zeros(27,3);
for k=1:27
    ant(k,:) = F.Rules(k).Antecedent;
end
assert(size(unique(ant,'rows'),1)==27);
[a,b,c] = ndgrid(1:3,1:3,1:3);
assert(isequal(sortrows(ant),sortrows([a(:) b(:) c(:)])));
disp('[OK] Estructura FIS y 27/27 antecedentes comprobados.');

%% 4. Concordancia de CSV con MAT
B = readtable('evidencias/A2_F1_barrido_1331.csv');
C = readtable('evidencias/A2_F1_casos_criticos.csv');
R = readtable('A2_F1_tabla_27_reglas.csv');
assert(height(B)==1331 && height(C)==10 && height(R)==27);

XB = B{:,{'df','di','dd'}};
YB = B{:,{'v','omega'}};
XC = C{:,{'df','di','dd'}};
YC = C{:,{'v','omega'}};

tolCSV = 1e-10;
assert(max(abs(XB-M2.X),[],'all')<tolCSV);
assert(max(abs(YB-M2.Y),[],'all')<tolCSV);
assert(max(abs(XC-M2.casos),[],'all')<tolCSV);
assert(max(abs(YC-M2.Z),[],'all')<tolCSV);
assert(size(unique(XB,'rows'),1)==1331);
disp('[OK] CSV consistentes con MAT (1331 barrido + 10 casos).');

%% 5. Reproduccion de inferencia desde FIS
op = evalfisOptions(NumSamplePoints=1001);
YR = evalfis(F,M2.X,op);
ZR = evalfis(F,M2.casos,op);
errBarrido = max(abs(YR-M2.Y),[],'all');
errCriticos = max(abs(ZR-M2.Z),[],'all');
tolEvaluacion = 1e-8;
assert(errBarrido<tolEvaluacion,'Barrido no reproducible.');
assert(errCriticos<tolEvaluacion,'Casos criticos no reproducibles.');
assert(all(isfinite(YR),'all') && all(isfinite(ZR),'all'));
assert(all(YR(:,1)>=0 & YR(:,1)<=0.2));
assert(all(YR(:,2)>=-1.5 & YR(:,2)<=1.5));

fprintf('[OK] Reproduccion barrido, error maximo: %.3g\n',errBarrido);
fprintf('[OK] Reproduccion criticos, error maximo: %.3g\n',errCriticos);

%% 6. Nueve PNG y lectura pixel a pixel
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
imagenes = dir(fullfile('evidencias','*.png'));
assert(numel(imagenes)==9,'Deben existir exactamente nueve PNG.');
for k = 1:numel(nombresPNG)
    ruta = fullfile('evidencias',nombresPNG(k));
    assert(isfile(ruta),'Falta PNG: %s',ruta);
    meta = imfinfo(ruta);
    img = imread(ruta);
    assert(~isempty(img) && meta.Width>0 && meta.Height>0, ...
        'PNG invalido: %s',ruta);
end
disp('[OK] Nueve PNG leidos y comprobados.');

%% 7. Seguridad documental y limitaciones
vEncerrado = evalfis(F,[0.2 0.2 0.2],op);
fprintf('[INFO] Tres lados cerca: v=%.9f m/s, omega=%+.9f rad/s\n', ...
    vEncerrado(1),vEncerrado(2));
if vEncerrado(1)>1e-6
    disp('[ADVERTENCIA] No hay parada estricta solo con Mamdani.');
    disp('             Parada de seguridad corresponde al supervisor ROS 2.');
end

disp('=== AUDITORIA COMPLEMENTARIA APROBADA EN MATLAB ===');
disp('ALCANCE: carga MAT, integridad de archivos y reproducibilidad FIS.');
disp('NO ACREDITA: saturacion fisica, ROS 2, comparacion Python, Gazebo.');
