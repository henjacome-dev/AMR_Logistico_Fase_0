
%% AVANCE 2 - FASE 1.5
% Superficies de control sobre el FIS validado v01.
% No modifica funciones de membresia ni reglas.

clearvars; clc; close all;

fis = readfis("AMR_Mamdani_A2_v01.fis");

assert(numel(fis.Rules)==27);

if ~isfolder("evidencias")
    mkdir("evidencias");
end

% Cada superficie representa 2 entradas variables.
% La tercera entrada se fija en un valor documentado.
%
% Entradas: 1=df, 2=di, 3=dd
% Salidas: 1=v, 2=omega

pares = {
    [1 2]
    [2 3]
    [2 3]
    [1 3]
};

salidas = [1; 2; 2; 1];

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

entradaNombres = ["df","di","dd"];
salidaNombres = ["v","omega"];

for k = 1:numel(nombres)

    opt = gensurfOptions;
    opt.InputIndex = pares{k};
    opt.OutputIndex = salidas(k);
    opt.ReferenceInputs = referencias{k};
    opt.NumGridPoints = 31;
    opt.NumSamplePoints = 1001;

    [X,Y,Z] = gensurf(fis,opt);

    assert(all(isfinite(Z),"all"), ...
        "La superficie tiene resultados invalidos.");

    if salidas(k)==1
        assert(all(Z>=-1e-9 & Z<=0.20+1e-9,"all"));
    else
        assert(all(abs(Z)<=1.5+1e-9,"all"));
    end

    f = figure("Name",nombres(k));

    surf(X,Y,Z);
    grid on;
    colorbar;
    view(45,30);

    xlabel(entradaNombres(pares{k}(1))+" (m)");
    ylabel(entradaNombres(pares{k}(2))+" (m)");

    if salidas(k)==1
        zlabel("v (m/s)");
    else
        zlabel("omega (rad/s)");
    end

    title(strrep(nombres(k),"_"," "));

    exportgraphics(f, ...
        fullfile("evidencias",nombres(k)+".png"), ...
        Resolution=160);

    fprintf("[OK] %s\n",nombres(k));
    fprintf("     Salida minima: %.6f\n",min(Z(:)));
    fprintf("     Salida maxima: %.6f\n",max(Z(:)));

    close(f);
end

disp("=== ETAPA 1.5: SUPERFICIES GENERADAS ===");
disp("[OK] Cuatro superficies calculadas.");
disp("[OK] Sin valores invalidos.");
disp("[OK] Limites de salida respetados.");
disp("[OK] Cuatro imagenes PNG guardadas.");
