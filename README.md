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
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake -DCMAKE_BUILD_TYPE=Release

# Compila il progetto
cmake --build . --config Release   
```

la parte `C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake` è un segnaposto. Ogni utente deve sostituirlo con il percorso reale della propria installazione di vcpkg sul proprio PC.

## ✅ Esecuzione del progetto

dopo aver completato la build, l'eseguibile del progetto si trova nella cartella: `build/Release`  (o `build/Debug` se hai compilato in modalità Debug).
Per avviare la simulazione, basta lanciare l'eseguibile
