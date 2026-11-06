
%% A2 - REPARACION DE EVIDENCIAS PNG
% No modifica el controlador ni las 27 reglas.

clearvars; clc; close all;

fis = readfis("AMR_Mamdani_A2_v01.fis");

assert(numel(fis.Inputs)==3);
assert(numel(fis.Outputs)==2);
assert(numel(fis.Rules)==27);

carpeta = "evidencias_reparadas";

if ~isfolder(carpeta)
    mkdir(carpeta);
end

%% PRUEBA PREVIA: escritura de un PNG independiente

ruta = fullfile(carpeta,"prueba_escritura.png");
imagen = uint8(randi([0 255],64,64,3));

imwrite(imagen,ruta,"png");
imfinfo(ruta);
imread(ruta);

disp("[OK] Escritura PNG independiente validada.");
delete(ruta);

%% 1. FUNCIONES DE MEMBRESIA

entradas = ["df","di","dd"];
salidas = ["v","omega"];

nombres = strings(9,1);
contador = 0;

for k = 1:5

    if k <= 3
        nombre = "entrada_"+entradas(k)+".png";
        tipo = "input";
        indice = k;
    else
        indice = k-3;
        nombre = "salida_"+salidas(indice)+".png";
        tipo = "output";
    end

    f = figure("Name",nombre,"Color","w", ...
        "Position",[100 100 900 650]);

    plotmf(fis,tipo,indice);
    grid on;
    title(strrep(nombre,"_"," "));

    drawnow;

    % Capturar pixeles sin usar exportgraphics
    captura = getframe(f);

    assert(~isempty(captura.cdata), ...
        "No se pudo capturar la figura.");

    ruta = fullfile(carpeta,nombre);
    imwrite(captura.cdata,ruta,"png");

    imfinfo(ruta);
    imread(ruta);

    contador = contador+1;
    nombres(contador) = nombre;

    fprintf("[OK] %s\n",nombre);
    close(f);
end

%% 2. SUPERFICIES DE CONTROL

pares = {
    [1 2]
    [2 3]
    [2 3]
    [1 3]
};

indicesSalida = [1;2;2;1];

referencias = {
    [NaN NaN 1.7]
    [0.2 NaN NaN]
    [1.7 NaN NaN]
    [NaN 0.9 NaN]
};

nombresSuperficie = [
    "superficie_v_df_di_dd_lejos"
    "superficie_omega_di_dd_df_cerca"
    "superficie_omega_di_dd_df_lejos"
    "superficie_v_df_dd_di_media"
];

for k = 1:4

    opt = gensurfOptions;
    opt.InputIndex = pares{k};
    opt.OutputIndex = indicesSalida(k);
    opt.ReferenceInputs = referencias{k};
    opt.NumGridPoints = 31;
    opt.NumSamplePoints = 1001;

    [X,Y,Z] = gensurf(fis,opt);

    assert(all(isfinite(Z),"all"), ...
        "Superficie con valores invalidos.");

    f = figure("Color","w", ...
        "Position",[100 100 900 650]);

    surf(X,Y,Z);
    grid on;
    colorbar;
    view(45,30);

    xlabel(entradas(pares{k}(1))+" (m)");
    ylabel(entradas(pares{k}(2))+" (m)");

    if indicesSalida(k)==1
        zlabel("v (m/s)");
    else
        zlabel("omega (rad/s)");
    end

    title(strrep(nombresSuperficie(k),"_"," "));

    drawnow;

    captura = getframe(f);

    assert(~isempty(captura.cdata), ...
        "No se pudo capturar la superficie.");

    nombre = nombresSuperficie(k)+".png";
    ruta = fullfile(carpeta,nombre);

    imwrite(captura.cdata,ruta,"png");

    imfinfo(ruta);
    imread(ruta);

    contador = contador+1;
    nombres(contador) = nombre;

    fprintf("[OK] %s\n",nombre);
    close(f);
end

%% 3. VALIDACION FINAL

disp("=== VALIDACION DE EVIDENCIAS ===");

for k = 1:numel(nombres)

    ruta = fullfile(carpeta,nombres(k));

    info = imfinfo(ruta);
    img = imread(ruta);

    assert(~isempty(img));

    fprintf("[OK] %s: %d x %d\n", ...
        nombres(k),info.Width,info.Height);
end

assert(contador==9);

disp("=== RESULTADO ===");
disp("[OK] 9 de 9 imagenes verificadas.");
disp("[OK] Controlador original intacto.");
disp("[OK] Evidencias en carpeta nueva.");
