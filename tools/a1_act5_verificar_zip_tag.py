#!/usr/bin/env python3
"""Actividad 5 A1: auditoría de ZIP Blackboard contra Git original y tag avance1.

Python estándar; no modifica el repositorio ni sus tags. Con --export-csv
produce un CSV detallado para registrar la comparación.

Ejemplo desde la raíz del repositorio en WSL:
python3 tools/a1_act5_verificar_zip_tag.py \
    /mnt/c/Users/Henry/Downloads/AMR_Logistico_Fase_1-main.zip \
    --export-csv docs/A1_ACT5_COMPARACION_ARCHIVOS.csv
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import os
import subprocess
import sys
import zipfile
from collections import Counter
from pathlib import Path

ZIP_SHA256 = "398835963816b9cf429ab68283b27340d6481fffa5006b115b6cad4cfc21502b"
COMMIT_ENTREGADO = "d1558d7e8845acf453a3b6eee5a1c62af947b3cf"
TAG_CONSOLIDADO = "avance1"
COMMIT_TAG = "db4e4b4f214bfe7cbe1e19192879a1890f1bcb02"
EXPECTED = {"original_archivos": 76, "tag_archivos": 156,
            "identicos": 66, "modificados": 10,
            "solo_tag": 80, "solo_zip": 0}


def git(repo: Path, *args: str) -> bytes:
    cmd = ["git", "-C", str(repo), *args]
    proc = subprocess.run(cmd, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE, check=False)
    if proc.returncode != 0:
        err = proc.stderr.decode("utf-8", "replace").strip()
        raise RuntimeError(f"{' '.join(cmd)}: {err}")
    return proc.stdout


def git_tree(repo: Path, ref: str) -> dict[str, str]:
    records = git(repo, "ls-tree", "-r", "-z", "--full-tree", ref)
    ret = {}
    for record in records.split(b"\x00"):
        if not record:
            continue
        meta, filename = record.split(b"\t", 1)
        mode, typ, sha = meta.decode("ascii").split(" ")
        if typ != "blob":
            continue
        # Git en este repositorio utiliza SHA-1.
        name = os.fsdecode(filename)
        if name in ret:
            raise ValueError(f"Ruta repetida en Git: {name}")
        ret[name] = sha
    return ret


def zip_tree(zip_path: Path):
    with zipfile.ZipFile(zip_path, "r") as z:
        names = z.namelist()
        bad = z.testzip()
        if bad:
            raise ValueError(f"CRC incorrecto o archivo dañado: {bad}")
        files = [i for i in z.infolist() if not i.is_dir()]
        if not files:
            raise ValueError("El ZIP no contiene archivos")
        prefixes = {i.filename.split("/", 1)[0] for i in files}
        if len(prefixes) == 1 and all("/" in i.filename for i in files):
            prefix = next(iter(prefixes)) + "/"
        else:
            prefix = ""
        ret = {}
        for item in files:
            path = item.filename[len(prefix):]
            if not path or path.startswith("/") or any(x == ".." for x in path.split("/")):
                raise ValueError(f"Ruta ZIP potencialmente insegura: {item.filename}")
            if path in ret:
                raise ValueError(f"Ruta ZIP duplicada: {path}")
            content = z.read(item)
            ret[path] = hashlib.sha1(
                f"blob {len(content)}\x00".encode() + content
            ).hexdigest()
        return ret, prefix, len(z.infolist()), sum(i.file_size for i in files)


def write_csv(path: Path, names: list[str], z: dict, o: dict, t: dict):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f, lineterminator="\n")
        writer.writerow(["ruta", "estado_zip_vs_avance1", "git_sha1_zip",
                         "git_sha1_commit_entregado", "git_sha1_tag_avance1"])
        for name in names:
            if name not in z:
                estado = "solo_tag_avance1"
            elif name not in t:
                estado = "solo_zip_blackboard"
            elif z[name] != t[name]:
                estado = "modificado_desde_entrega"
            else:
                estado = "identico"
            writer.writerow([name, estado, z.get(name, ""), o.get(name, ""), t.get(name, "")])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("zip", type=Path, help="ZIP A1 original, descargado de Blackboard")
    parser.add_argument("--repo", type=Path,
                        default=Path(__file__).resolve().parent.parent,
                        help="Raíz de la copia Git (por defecto: carpeta superior a tools)")
    parser.add_argument("--export-csv", type=Path, help="Escribe comparación por archivo")
    args = parser.parse_args()

    repo, zip_path = args.repo.resolve(), args.zip.expanduser().resolve()
    if not zip_path.is_file():
        parser.error(f"ZIP no encontrado: {zip_path}")
    if not (repo / ".git").exists():
        parser.error(f"No se encontró .git en {repo}")
    print("=== A1 / ACTIVIDAD 5: BLACKBOARD VS GIT ===")
    print("Repositorio:", repo)
    print("ZIP:", zip_path)
    actual_sha = hashlib.sha256(zip_path.read_bytes()).hexdigest()
    print("ZIP SHA256:", actual_sha)
    print("[OK] SHA-256 de ZIP" if actual_sha == ZIP_SHA256 else
          "[ALERTA] SHA-256 de ZIP distinto de la referencia")

    zip_blobs, root, entries, raw_size = zip_tree(zip_path)
    print(f"ZIP: {entries} entradas; {len(zip_blobs)} archivos; {raw_size} bytes extraídos")
    print("Carpeta raíz ZIP:", root or "(sin carpeta común)")

    commit_original = git(repo, "rev-parse", f"{COMMIT_ENTREGADO}^{{commit}}").decode().strip()
    commit_tag = git(repo, "rev-parse", f"{TAG_CONSOLIDADO}^{{commit}}").decode().strip()
    print("Commit original:", commit_original)
    print("Tag avance1 →:", commit_tag)

    tree_original = git_tree(repo, COMMIT_ENTREGADO)
    tree_tag = git_tree(repo, TAG_CONSOLIDADO)
    names = sorted(set(zip_blobs) | set(tree_original) | set(tree_tag))

    original_added = sorted(set(tree_original) - set(zip_blobs))
    original_missing = sorted(set(zip_blobs) - set(tree_original))
    original_changed = sorted(k for k in zip_blobs if k in tree_original and zip_blobs[k] != tree_original[k])
    full_identity = not (original_added or original_missing or original_changed)
    print(f"ZIP ↔ commit entregado: {len(zip_blobs)} ↔ {len(tree_original)} archivos")
    print("[OK] Coincidencia exacta de todos los contenidos" if full_identity else
          "[ALERTA] El ZIP difiere del commit histórico")
    if not full_identity:
        for cat, group in [("Solo commit", original_added), ("Solo ZIP", original_missing),
                           ("Contenido diferente", original_changed)]:
            for p in group:
                print("  ", cat, p)

    equal = sorted(k for k in zip_blobs if k in tree_tag and zip_blobs[k] == tree_tag[k])
    changed = sorted(k for k in zip_blobs if k in tree_tag and zip_blobs[k] != tree_tag[k])
    only_tag = sorted(set(tree_tag) - set(zip_blobs))
    only_zip = sorted(set(zip_blobs) - set(tree_tag))
    values = {"original_archivos": len(zip_blobs), "tag_archivos": len(tree_tag),
              "identicos": len(equal), "modificados": len(changed),
              "solo_tag": len(only_tag), "solo_zip": len(only_zip)}
    print("ZIP ↔ tag avance1:", values)
    for k in changed:
        print("  MODIFICADO", k)
    extra_types = Counter(p.split("/", 1)[0] for p in only_tag)
    print("Nuevos en avance1 por carpeta:", dict(sorted(extra_types.items())))
    if args.export_csv:
        out = args.export_csv if args.export_csv.is_absolute() else (repo / args.export_csv)
        write_csv(out, names, zip_blobs, tree_original, tree_tag)
        print("CSV generado:", out)
    consistent = (actual_sha == ZIP_SHA256 and full_identity and
                  commit_original == COMMIT_ENTREGADO and commit_tag == COMMIT_TAG and
                  values == EXPECTED)
    print("[RESULTADO] AUDITORÍA APROBADA" if consistent else
          "[RESULTADO] REVISAR: una o más expectativas no coinciden")
    return 0 if consistent else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, ValueError, RuntimeError, zipfile.BadZipFile) as e:
        print("[ERROR]", str(e), file=sys.stderr)
        print("Sugerencia: 'git fetch origin --tags' en la copia local", file=sys.stderr)
        sys.exit(2)
