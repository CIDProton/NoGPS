#!/usr/bin/env python3
# -*- coding: utf-8 -*-

#░█████╗░██╗██████╗░░░░░░░░██████╗███╗░░██╗██████╗░
#██╔══██╗██║██╔══██╗░░░░░░██╔════╝████╗░██║██╔══██╗
#██║░░╚═╝██║██║░░██║█████╗╚█████╗░██╔██╗██║██████╔╝
#██║░░██╗██║██║░░██║╚════╝░╚═══██╗██║╚████║██╔═══╝░
#╚█████╔╝██║██████╔╝░░░░░░██████╔╝██║░╚███║██║░░░░░
#░╚════╝░╚═╝╚═════╝░░░░░░░╚═════╝░╚═╝░░╚══╝╚═╝░░░░░
#CID-Soft | SNP — Source sNaPshot

# ---------------- CONFIG ----------------

AI_MODE = True
DEFAULT_PREFIX = "nogps-test-ai"
BACKUP_DIR_NAME = "dump"
MAX_FILE_SIZE = 1_000_000  # 1 MB
SKIP_DIRS = {".git", "__pycache__", "venv", "venv_linux", "env", BACKUP_DIR_NAME}
SKIP_EXTENSIONS = {".pyc", ".log"}

# --------------- IMPORTS ---------------

import os, re, datetime, argparse

# ---------------- UTILS ----------------

def get_version_from_dir():
    name = os.path.basename(os.getcwd())
    v = re.sub(r"[^\d.]", "", name)
    return v if v else "dev"

def generate_filename(prefix):
    now = datetime.datetime.now()
    date = now.strftime("%Y%m%d")
    time = now.strftime("%H%M%S")
    version = get_version_from_dir()
    return f"{prefix}-{version}-{date}-{time}-dump.md"

def should_skip(path):
    base = os.path.basename(path)

    if base.startswith(".") or base == os.path.basename(__file__):
        return True

    if os.path.isdir(path) and base in SKIP_DIRS:
        return True

    if os.path.isfile(path):
        ext = os.path.splitext(base)[1]
        if ext in SKIP_EXTENSIONS:
            return True

    return False

# ---------------- CORE ----------------

def dump_project(base_dir, output_file):
    files_count = 0
    total_size = 0

    with open(output_file, "w", encoding="utf-8") as out:
        out.write(f"# PROJECT DUMP\n")
        out.write(f"# Directory: {base_dir}\n")
        out.write(f"# Generated: {datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        out.write("=" * 80 + "\n\n")

        for root, dirs, files in os.walk(base_dir):
            dirs[:] = [d for d in dirs if not should_skip(os.path.join(root, d))]

            for file in sorted(files):
                full_path = os.path.join(root, file)

                if should_skip(full_path):
                    continue

                try:
                    size = os.path.getsize(full_path)
                    if size > MAX_FILE_SIZE:
                        continue

                    with open(full_path, "r", encoding="utf-8") as f:
                        content = f.read()

                except Exception:
                    continue

                rel_path = os.path.relpath(full_path, base_dir)

                out.write("\n" + "=" * 80 + "\n")
                out.write(f"FILE: {rel_path}\n")
                out.write("=" * 80 + "\n\n")
                
                if AI_MODE == True:
                    ext = os.path.splitext(rel_path)[1].lstrip('.')
                    out.write(f"```{ext}\n")
                    out.write(content.rstrip() + "\n")
                    out.write("```\n")
                else:
                    out.write(content.rstrip() + "\n")

                files_count += 1
                total_size += len(content)

        out.write("\n" + "=" * 80 + "\n")
        out.write("STATS\n")
        out.write(f"Files: {files_count}\n")
        out.write(f"Total text size: {total_size} bytes\n")
        out.write("=" * 80 + "\n")

    return files_count, total_size

# ---------------- MAIN ----------------

def main():
    parser = argparse.ArgumentParser(description="Dump full project into one file")
    parser.add_argument("-n", "--name", help="Custom output filename")
    parser.add_argument("-p", "--prefix", default=DEFAULT_PREFIX)
    args = parser.parse_args()

    base_dir = os.getcwd()
    dump_dir = os.path.join(base_dir, BACKUP_DIR_NAME)
    os.makedirs(dump_dir, exist_ok=True)

    if args.name:
        output_path = os.path.join(dump_dir, args.name)
    else:
        output_path = os.path.join(dump_dir, generate_filename(args.prefix))

    print(f"[+] Scanning: {base_dir}")
    print(f"[+] Output:   {output_path}")

    files, size = dump_project(base_dir, output_path)

    print("\n✅ DONE")
    print(f"Files dumped: {files}")
    print(f"Total size:   {size} bytes")
    print(f"Saved as:     {output_path}")

if __name__ == "__main__":
    main()