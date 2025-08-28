import argparse
import os
import re
import subprocess
from pathlib import Path
import cv2

def create_video_with_opencv(frames_dir, output_file, fps=24):
    frames = sorted(Path(frames_dir).glob("frame_*.ppm"))
    if not frames:
        raise RuntimeError(f"Nessun frame trovato in {frames_dir}!")

    # Leggi il primo frame per prendere dimensioni
    first_frame = cv2.imread(str(frames[0]))
    if first_frame is None:
        raise RuntimeError("Impossibile leggere il primo frame.")
    height, width, _ = first_frame.shape

    # Inizializza il writer
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")  # Codec compatibile MP4
    writer = cv2.VideoWriter(str(output_file), fourcc, fps, (width, height))

    print(f"[INFO] Creo video {output_file} con {len(frames)} frame ({width}x{height}, {fps} FPS)")

    for frame_path in frames:
        img = cv2.imread(str(frame_path))
        if img is None:
            print(f"[WARN] Impossibile leggere {frame_path}, salto.")
            continue
        img = cv2.flip(img, 0)  # vflip
        writer.write(img)

    writer.release()
    print(f"[INFO] Video creato: {output_file}")


# Percorsi principali
ROOT_DIR         = Path(__file__).parent.resolve()
SETTINGS_FILE    = ROOT_DIR  / "settings.cpp"
BUILD_DIR        = ROOT_DIR  / "build"
DEBUG_DIR        = BUILD_DIR / "Debug"
RELEASE_DIR      = BUILD_DIR / "Release"
VIDEO_FRAMES_DIR = ROOT_DIR  / "video_frame"

# Regex per modificare le variabili nel file C++
patterns = {
    "DO_VIDEO": re.compile(r"(bool\s+DO_VIDEO\s*=\s*)(true|false)")
}

def update_settings(do_video):
    content = SETTINGS_FILE.read_text()

    content = patterns["DO_VIDEO"].sub(rf"\1{'true' if do_video else 'false'}", content)

    SETTINGS_FILE.write_text(content)
    print("[INFO] settings.cpp aggiornato")

def run_command(command, cwd=None):
    print(f"[RUN] {' '.join(command)} (cwd={cwd})")
    result = subprocess.run(command, cwd=cwd)
    if result.returncode != 0:
        raise RuntimeError(f"Comando fallito: {' '.join(command)}")

def main():
    parser = argparse.ArgumentParser(description="Launcher per XPBDpallet")
    parser.add_argument("--novideo", action="store_true", help="Disabilita output video")
    args = parser.parse_args()

    # 1. Aggiorna settings.cpp
    update_settings(not args.novideo)

    # 2. Build del progetto
    run_command(["cmake", "--build", ".", "--config", "Release"], cwd=BUILD_DIR)

    # 3. Esegui simulazione
    exe_path = RELEASE_DIR / "XPBDPallet.exe"
    run_command([str(exe_path)], cwd=RELEASE_DIR)

    # 4. Genera video con ffmpeg
    if not args.novideo:
        if not VIDEO_FRAMES_DIR.exists():
            raise RuntimeError(f"Cartella {VIDEO_FRAMES_DIR} non trovata!")
        output_file = VIDEO_FRAMES_DIR / "output.mp4"
        create_video_with_opencv(VIDEO_FRAMES_DIR, output_file, fps=24)
        print(f"[INFO] Video generato: {output_file}")

if __name__ == "__main__":
    main()
