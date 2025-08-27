## ✅ Prerequisiti

- **CMake** >= 3.15
- **Visual Studio** (o un compilatore C++ compatibile)
- [**vcpkg**](https://github.com/microsoft/vcpkg) installato e configurato

---

## ✅ Installazione dipendenze

Per gestire le dipendenze utilizziamo **vcpkg**. Se non lo hai ancora installato:

```bash
# Clona vcpkg
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg

# Compila vcpkg
bootstrap-vcpkg.bat    # Su Windows
```

---

## ✅ Installazione pacchetti

Installa i pacchetti necessari con **vcpkg**:

```bash
vcpkg install glfw3 glad glm
```

---

<!-- ## ✅ Settaggio percorsi dei pacchetti

Nel file `CMakeLists.txt` sono presenti i percorsi per trovare le librerie installate tramite **vcpkg**:

```cmake
set(glfw3_DIR "C:/Users/Workstation/vcpkg/installed/x64-windows/share/glfw3")
set(glad_DIR "C:/Users/Workstation/vcpkg/installed/x64-windows/share/glad")
set(glm_DIR "C:/Users/Workstation/vcpkg/installed/x64-windows/share/glm")
```

Questi percorsi devono essere modificati in base alla posizione della cartella vcpkg sul tuo PC.

--- -->

## ✅ Build del progetto

```bash
# Crea una cartella per i file di build
mkdir build
cd build

# Configura il progetto con CMake
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake

# Compila il progetto
cmake --build .
```

la parte `C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake` è un segnaposto. Ogni utente deve sostituirlo con il percorso reale della propria installazione di vcpkg sul proprio PC.

## ✅ Esecuzione del progetto con il Launcher

L'intero processo (configurazione, build, simulazione e generazione video) è gestito dal launcher in Python. Requisiti

- **Python 3**.
- Libreria **OpenCV**:    `pip install opencv-python`

Dal root del progetto, esegui:

```bash
python launcher.py [OPZIONI]
```

Opzioni disponibili:

- `--novideo` → Disabilita la generazione del video.

## ✅ Esecuzione del progetto senza Launcher

alterntivamente, dopo aver completato la build, l'eseguibile del progetto si trova nella cartella: `build/Debug` (o `build/Release` se hai compilato in modalità Release).
Per avviare la simulazione, basta lanciare l'eseguibile.

## ✅ Generazione manuale del video

Per registrare un video della simulazione:

1. Assicurati di avere **FFmpeg** installato sul tuo sistema e accessibile dal PATH.Puoi scaricarlo da: [https://ffmpeg.org/download.html](https://ffmpeg.org/download.html)
2. Nel codice sorgente, abilita la registrazione del video impostando la variabile:

   ```cpp
   bool DO_VIDEO = true;
   ```
   e poi compila il progetto.
3. Esegui la simulazione; i frame verranno salvati nella cartella `video_frame` come immagini:
   frame_00001.ppm, frame_00002.ppm, ecc.
4. Per creare il video finale, apri un terminale nella cartella `video_frame` e lancia il comando:

   ```bash
   ffmpeg -framerate 24 -i frame_%05d.ppm -vf "vflip" -c:v libx264 -pix_fmt yuv420p output.mp4
   ```
