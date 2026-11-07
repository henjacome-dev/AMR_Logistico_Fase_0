%% Avance 2 - Fase 1.2
% Controlador Mamdani: definicion de variables y membresias
% Aun no se agregan reglas; se incorporaran en Fase 1.3.

clearvars; clc; close all;

fis = mamfis( ...
    Name="AMR_Mamdani_A2", ...
    AndMethod="min", ...
    OrMethod="max", ...
    ImplicationMethod="min", ...
    AggregationMethod="max", ...
    DefuzzificationMethod="centroid");

%% Entradas: distancias frontal, izquierda y derecha

entradas = ["df", "di", "dd"];

for nombre = entradas
    fis = addInput(fis,[0 2],Name=nombre);

    fis = addMF(fis,nombre,"trapmf", ...
        [0 0 0.30 0.70],Name="Cerca");

    fis = addMF(fis,nombre,"trimf", ...
        [0.40 0.90 1.40],Name="Media");

    fis = addMF(fis,nombre,"trapmf", ...
        [1.10 1.50 2 2],Name="Lejos");
end

%% Salida: velocidad lineal (m/s)

fis = addOutput(fis,[0 0.20],Name="v");

fis = addMF(fis,"v","trapmf", ...
    [0 0 0.005 0.035],Name="Detenido");

fis = addMF(fis,"v","trimf", ...
    [0.020 0.070 0.120],Name="Lenta");

fis = addMF(fis,"v","trapmf", ...
    [0.100 0.150 0.200 0.200],Name="Rapida");

%% Salida: velocidad angular (rad/s)
% Positivo: izquierda; negativo: derecha (REP-103)

fis = addOutput(fis,[-1.5 1.5],Name="omega");

fis = addMF(fis,"omega","trapmf", ...
    [-1.5 -1.5 -1.05 -0.65],Name="DerechaFuerte");

fis = addMF(fis,"omega","trimf", ...
    [-0.90 -0.50 -0.15],Name="DerechaSuave");

fis = addMF(fis,"omega","trimf", ...
    [-0.22 0 0.22],Name="Recto");

fis = addMF(fis,"omega","trimf", ...
    [0.15 0.50 0.90],Name="IzquierdaSuave");

fis = addMF(fis,"omega","trapmf", ...
    [0.65 1.05 1.5 1.5],Name="IzquierdaFuerte");

%% Validacion estructural

assert(numel(fis.Inputs)==3);
assert(numel(fis.Outputs)==2);

for k = 1:3
    assert(numel(fis.Inputs(k).MembershipFunctions)==3);

    [~,mu] = plotmf(fis,"input",k,401);
    assert(all(max(mu,[],2)>0), ...
        "Entrada sin cobertura de membresia");
end

assert(numel(fis.Outputs(1).MembershipFunctions)==3);
assert(numel(fis.Outputs(2).MembershipFunctions)==5);

for k = 1:2
    [~,mu] = plotmf(fis,"output",k,401);
    assert(all(max(mu,[],2)>0), ...
        "Salida sin cobertura de membresia");
end

assert(isempty(fis.Rules), ...
    "Todavia no deben existir reglas");

disp("=== VALIDACION FASE 1.2 ===")
disp(fis)

fprintf("Entradas: %d\n",numel(fis.Inputs));
fprintf("Salidas: %d\n",numel(fis.Outputs));
fprintf("Reglas actuales: %d\n",numel(fis.Rules));
disp("[OK] Universos y membresias validados.");

close all; % cerrar figuras generadas por las pruebas plotmf

%% Generacion segura de evidencias (AC-01)
% NO escribe en evidencias/ ni sobrescribe A2_F1_membresias_base.mat.
carpeta = "evidencias_AC01_prueba";
if ~isfolder(carpeta)
    mkdir(carpeta);
end

nombres = ["entrada_df.png"; "entrada_di.png"; "entrada_dd.png"; ...
           "salida_v.png"; "salida_omega.png"];

for k = 1:5
    if k <= 3
        tipo = "input";
        indice = k;
        etiqueta = entradas(k);
    else
        tipo = "output";
        indice = k - 3;
        salidas = ["v", "omega"];
        etiqueta = salidas(indice);
    end

    ruta = fullfile(carpeta, nombres(k));
    assert(~isfile(ruta), "AC-01: ya existe %s. Conservelo o retirelo antes de repetir.", ruta);

    f = figure("Name", nombres(k), "Color", "w", ...
        "Position", [100 100 900 650]);
    plotmf(fis, tipo, indice);
    grid on;
    title("Membresias de " + etiqueta);
    drawnow;

    captura = getframe(f);
    assert(~isempty(captura.cdata), "Figura vacia: %s", nombres(k));
    imwrite(captura.cdata, ruta, "png");
    meta = imfinfo(ruta);
    imagen = imread(ruta);
    assert(~isempty(imagen) && meta.Width>0 && meta.Height>0, ...
        "PNG invalido: %s", ruta);
    fprintf("[OK] AC01 %s: %d x %d\n", nombres(k), meta.Width, meta.Height);
    close(f);
end

disp("=== AC01 MEMBRESIAS: 5 PNG GENERADOS Y LEIBLES ===");
disp("[OK] Sin modificaciones al FIS, MAT base ni evidencias oficiales.");
