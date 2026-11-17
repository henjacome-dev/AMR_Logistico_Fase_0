# Control de integridad Git — A3 / U3S7

**Proyecto:** AMR Logístico MEC451
**Repositorio:** `henjacome-dev/AMR_Logistico_Fase_0`
**Rama auditada:** `u3s7-entrega-final-2026-11-16`
**Archivo fuente:** `AMR_Logistico_U3S7_para_equipo.zip`

## 1. Identificación de la entrega

- **SHA-256 del ZIP original:** `dbd21e118c0bd3c1530c6dfa948be668765d072ed9be9d9fe81fabab0fc32da8`.
- **Integridad del ZIP:** `ZipFile.testzip() = None`, sin archivos con CRC inválido.
- **Contenido:** 1.245 archivos y 172 entradas de directorio; aproximadamente 153,2 MB descomprimidos.
- **Commit de importación:** `e38b177e373e3a68c2e77d478231b3e97f658f4b`.
- **Commit de la rama al momento de auditar:** `7f82c0b0e20626abe1b86a9341a8b36ed0957188`.

## 2. Correspondencia de archivos

La comparación de los manifiestos de rutas y hashes Git (`blob SHA-1`) del ZIP con el árbol del commit de importación confirma la correspondencia de sus **1.245 archivos**.

**Excepción de normalización:** `resultados/U3S7/tabla_ordenes.csv` fue convertido de finales de línea CRLF a LF para superar la comprobación `git diff --cached --check`. La conversión no altera los campos ni registros CSV, pero modifica sus bytes y su SHA Git.

- SHA Git de los bytes del CSV dentro del ZIP: `b23ebc8d9815ccd73a53a39d98d47b794cd49871`.
- SHA Git del CSV normalizado y publicado: `bb933df66c028e7499e303f76f04d99a9f28caa1`.

Con esa transformación aplicada, el manifiesto de contenido del ZIP y el árbol de `e38b177` coinciden. Por consiguiente, se acredita una **correspondencia funcional y de archivos**, no una igualdad binaria completa del CSV.

## 3. Modificaciones posteriores

La rama actual contiene 1.248 archivos: los 1.245 del commit de importación, más tres archivos nuevos:

1. `docs/CONTROL_ENTREGA_A1.md`.
2. `docs/CONTROL_ENTREGA_A2.md`.
3. `tools/amr_git.sh`.

No se detectaron archivos eliminados ni contenidos modificados respecto del árbol `e38b177` al comparar con la punta de rama `7f82c0b`.

## 4. Estado del tag `final` — VERIFICADO

**Estado:** completado y verificado en GitHub el 17 de noviembre de 2026.

- **Referencia remota:** `refs/tags/final`.
- **Tipo:** tag anotado de Git.
- **SHA del objeto tag:** `0ca1272f36f98df5c5064c4686c0c8e6b15bbc57`.
- **Commit al que apunta:** `e38b177e373e3a68c2e77d478231b3e97f658f4b`.
- **Mensaje del tag:** `Entrega final A3 U3S7 - AMR Logistico`.
- **Autor del tag:** `henjacome-dev`.

**Verificación del usuario en Ubuntu:**

```text
git push origin refs/tags/final
* [new tag] final -> final

git rev-parse 'final^{commit}'
e38b177e373e3a68c2e77d478231b3e97f658f4b

git ls-remote origin 'refs/tags/final^{}'
e38b177e373e3a68c2e77d478231b3e97f658f4b refs/tags/final^{}
```

**Verificación independiente:** la referencia y el objeto del tag remoto se consultaron desde GitHub, confirmando la identidad del commit. El SHA del objeto tag es distinto del SHA del commit etiquetado porque se trata de un tag anotado.

La rama de trabajo puede continuar recibiendo commits sin alterar este hito; no recrear ni desplazar el tag `final`.

## 5. Dictamen y cierre de la actividad 3

- Repositorio y commit de importación: **verificados**.
- Correspondencia de archivos ZIP–Git: **verificada con una excepción de formato CRLF→LF documentada**.
- Rama vigente: **verificada**, con tres archivos adicionales de gestión.
- Tag literal `final`: **creado y publicado; apunta a `e38b177`**.

Este documento es una auditoría retrospectiva. No modifica la fecha de entrega original ni constituye una nota oficial del docente.

**Conclusión de actividad 3 (A3): CERRADA.** Las tres comprobaciones —tag, commit y correspondencia con el ZIP— están acreditadas.
