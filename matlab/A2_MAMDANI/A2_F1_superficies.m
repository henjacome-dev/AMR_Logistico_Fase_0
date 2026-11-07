%% AC-05 - TRAZABILIDAD DE LAS CUATRO SUPERFICIES
% Solo crea PNG candidatos en evidencias_AC05_prueba/.
% No modifica el FIS v01 ni evidencias/ ni evidencias_AC01_prueba/.
% Reutiliza los parametros de calculo validados en AC-01.

clearvars; clc; close all;

fis = readfis("AMR_Mamdani_A2_v01.fis");
assert(numel(fis.Inputs)==3 && numel(fis.Outputs)==2);
assert(numel(fis.Rules)==27);

carpeta = "evidencias_AC05_prueba";
if ~isfolder(carpeta)
    mkdir(carpeta);
end

% Entradas: 1=df, 2=di, 3=dd. Salidas: 1=v, 2=omega.
pares = {
    [1 2]
    [2 3]
    [2 3]
    [1 3]
};

indicesSalida = [1; 2; 2; 1];
referencias = {
    [NaN NaN 1.7]
    [0.2 NaN NaN]
    [1.7 NaN NaN]
    [NaN 0.9 NaN]
};

nombres = [
    "superficie_v_df_di_dd_lejos"
    "superficie_omega_di_dd_df_cerca"
    "superficie_omega_di_dd_df_lejos"
    "superficie_v_df_dd_di_media"
];

nombreEntrada = ["df", "di", "dd"];
nombreSalida  = ["v", "omega"];

% Verificar que no se sobreescribiran PNG candidatos anteriores.
for k = 1:numel(nombres)
    ruta = fullfile(carpeta,nombres(k)+".png");
    assert(~isfile(ruta), ...
        "AC-05: ya existe %s. Cambie el nombre de la carpeta anterior antes de repetir.",ruta);
end

disp("=== AC-05: SUPERFICIES CON ENTRADA FIJA EXPLICITA ===");

for k = 1:numel(nombres)
    ref = referencias{k};
    par = pares{k};
    indiceFijo = setdiff(1:3,par);

    assert(numel(indiceFijo)==1);
    assert(all(isnan(ref(par))) && isfinite(ref(indiceFijo)));
    valorFijo = ref(indiceFijo);

    opt = gensurfOptions;
    opt.InputIndex = par;
    opt.OutputIndex = indicesSalida(k);
    opt.ReferenceInputs = ref;
    opt.NumGridPoints = 31;
    opt.NumSamplePoints = 1001;

    [X,Y,Z] = gensurf(fis,opt);
    assert(all(isfinite(Z),"all"), "Superficie con valores no finitos.");

    if indicesSalida(k)==1
        assert(all(Z>=-1e-9 & Z<=0.20+1e-9,"all"));
        etiquetaSalida = "Velocidad lineal v";
        etiquetaZ = "v (m/s)";
    else
        assert(all(abs(Z)<=1.5+1e-9,"all"));
        etiquetaSalida = "Velocidad angular omega";
        etiquetaZ = "omega (rad/s)";
    end

    f = figure("Name",nombres(k),"Color","w", ...
        "Position",[100 100 900 650]);
    ax = axes("Parent",f);
    surf(ax,X,Y,Z);
    grid(ax,"on");
    colorbar(ax);
    view(ax,45,30);

    xlabel(ax,nombreEntrada(par(1))+" (m)");
    ylabel(ax,nombreEntrada(par(2))+" (m)");
    zlabel(ax,etiquetaZ);

    titulo1 = sprintf("%s frente a %s y %s", ...
        char(etiquetaSalida),char(nombreEntrada(par(1))), ...
        char(nombreEntrada(par(2))));
    titulo2 = sprintf("Entrada fija: %s = %.1f m", ...
        char(nombreEntrada(indiceFijo)),valorFijo);
    title(ax,{titulo1,titulo2},"Interpreter","none","FontSize",11);
    drawnow;

    captura = getframe(f);
    assert(~isempty(captura.cdata),"No se pudo capturar %s",nombres(k));

    ruta = fullfile(carpeta,nombres(k)+".png");
    imwrite(captura.cdata,ruta,"png");

    info = imfinfo(ruta);
    img = imread(ruta);
    assert(info.Width>=600 && info.Height>=400 && ~isempty(img));
    assert(any(img(:)~=img(1)),"Imagen uniforme: %s",ruta);

    fprintf("[OK] %-46s %s=%.1f m; Z=[%.6f, %.6f]; %d x %d\n", ...
        nombres(k),char(nombreEntrada(indiceFijo)),valorFijo, ...
        min(Z(:)),max(Z(:)),info.Width,info.Height);
    close(f);
end

disp("=== AC-05: CUATRO IMAGENES CANDIDATAS GENERADAS ===");
disp("[OK] FIS y evidencias oficiales sin modificaciones.");
