# 🫧 Soft Body Physics

A real-time **Soft Body Physics simulation** built with **C++ and SFML 3.0**, featuring jelly-like objects that stretch, squish, and wobble using a mass-spring system.

![Soft Body Physics Preview](https://raw.githubusercontent.com/snehal-thombare08/Soft-Body-Physics/main/Screenshot%202026-06-17%20132208.png)

## ✨ Features

- **Mass-Spring System** — each blob is a grid of point masses connected by structural, shear, and bend springs
- **Realistic Deformation** — blobs stretch and squish on impact, then spring back
- **Gravity & Bouncing** — full gravity with energy loss on wall/floor collisions
- **Grab & Throw** — drag any blob with the mouse and fling it across the screen
- **Explosion Mode** — launch all blobs into the air simultaneously
- **Pin Points** — pin individual mass points to anchor blobs in mid-air
- **Spring Wireframe** — toggle spring visualization to see the internal structure
- **Multi-color Blobs** — 5 rotating colors, randomized grid sizes

## screenshot
https://raw.githubusercontent.com/snehal-thombare08/Soft-Body-Physics/main/Screenshot%202026-06-17%20132208.png

## 🛠️ Built With

- **C++17**
- **SFML 3.0.2**
- **CMake** + **MinGW**

## ▶️ How to Run

1. Download `SoftBody-v1.0-Windows.zip` from [Releases](../../releases)
2. Extract all files to the same folder
3. Run `SoftBody.exe`

> Requires Windows. No install needed — just extract and run.

## 🎮 Controls

| Key / Mouse | Action |
|---|---|
| **LMB Click** | Spawn new blob |
| **RMB Drag** | Grab and throw a blob |
| **Space** | Explode all blobs upward |
| **S** | Toggle spring wireframe |
| **P** | Pin/unpin nearest point |
| **C** | Clear all blobs |
| **Esc** | Quit |

## ⚙️ Build from Source

```bash
git clone https://github.com/snehal-thombare08/Soft-Body-Physics.git
cd Soft-Body-Physics
mkdir build && cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=C:/vcpkg/scripts/buildsystems/vcpkg.cmake -G "MinGW Makefiles"
mingw32-make
```

## 📁 Project Structure

```
Soft-Body-Physics/
├── src/
│   └── main.cpp       # Full simulation source
├── CMakeLists.txt
└── README.md
```

---

Part of a C++ graphics & simulation portfolio built with SFML 3.0.
