# MEC451 — Cierre A12–A16 y publicación A17

**FIS operativo oficial: v03**. FIS v04 se reserva para experimentación A13, sin transición automática de versión.

## Integridad

Los dictámenes de B1–B4, A13, A14 y A15 fueron comprobados contra sus manifiestos SHA256 en Ubuntu antes del commit. Los CSV publicados son copia de las fuentes (no reconstrucciones de ensayos). Se conserva el hash de cada dictamen y de cada manifiesto en `snapshot_A17.json`.

## Alcance de las pruebas

| Actividad | Estado observado | Reserva |
|---|---|---|
| A12 B1–B4 | 20/20; 0 contactos medidos | B2 A*, B3 híbrido, B4 asistido en sombra |
| A13 | 5 pares, 10 ensayos A/B acreditados | No se infiere superioridad de v04 ni Mamdani puro |
| A14 | 30/30 rosbag SQLite/SHA256; 6/6 replay aislado | B1: cinco grabaciones nuevas, no las históricas |
| A15 | 600 PNG originales, 600 derivados, 90/90 test | Solo dominio sintético Gazebo; CNN inicial, no MobileNetV2 |
| A16 | Dos ZIP locales, hashes coincidentes | Acceso del docente reportado por usuario; URL no publicada |

## Archivo original e inmutabilidad

- Tag `avance2` → commit `dbbac54debea339d9ff9cd14985b66f9ada44c16`.
- FIS v03 SHA256 `67dc69177a8e227f84e333977b9366fc57d07c79a37af2d0ae2d5dad56a41903`.
- Esta rama documental no cambia `.fis`, lanzamiento, mundo, sensores o controladores.
- Rosbags, PNG originales y pesos .keras completos permanecen localmente. No se incorpora un enlace inexistente.
- El CI de GitHub valida la consistencia de la **copia publicada**, no reproduce Gazebo, MATLAB ni la CNN.

**Fecha de acta:** 2026-11-09T18:50:11.320930+00:00
