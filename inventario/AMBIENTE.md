# Ambiente rilevato

Versioni lette il 26 ottobre 2026 sulla macchina Ubuntu dove sono state eseguite le campagne
(`lsb_release`, `uname`, `dpkg-query`, `cmake --version`, `g++ --version`, `python3 --version`,
`ign gazebo --versions`). Nessuna dipendenza è stata copiata: vanno installate dai pacchetti.

## Sistema e strumenti

| Componente | Versione |
|---|---|
| Ubuntu | 22.04.5 LTS (jammy), kernel 6.8.0-138-generic |
| CPU / RAM | Intel Core i7-6700 @ 3.40 GHz / 15 GiB |
| CMake | 3.22.1 |
| g++ | 11.4.0 (Ubuntu 11.4.0-1ubuntu1~22.04.3) |
| Python | 3.10.12 (numpy 1.21.5, matplotlib 3.5.1, PyYAML 5.4.1) |
| colcon | `python3-colcon-common-extensions` 0.3.0, `colcon-core` 0.20.1 |

## ROS 2

| Pacchetto | Versione |
|---|---|
| ROS 2 **Humble** (`ros-humble-desktop`, `ros-base`, `ros-core`) | 0.10.0 (build 2026-03-26) |
| `ros-humble-rclcpp` | 16.0.19 |
| `ros-humble-sensor-msgs`, `std-msgs` | 4.9.1 |
| `ros-humble-tf2-msgs` | 0.25.20 |
| `ros-humble-ros-gz` (`-bridge`, `-interfaces`, `-sim`) | 0.244.23 |
| `ros-humble-ament-cmake` | 1.3.14 |
| RMW | `ros-humble-rmw-fastrtps-cpp` 6.2.10 (unico installato) |

Dipendenze dichiarate da `src/collision_avoidance/package.xml`: `ament_cmake`, `rclcpp`,
`sensor_msgs`, `std_msgs`, `tf2_msgs`, `ros_gz_interfaces` (test: `ament_lint_auto`,
`ament_lint_common`). Gli script Python di controllo (`scripts/check_*.py`) usano anche `rclpy`
e `geometry_msgs`.

## Gazebo

| Pacchetto | Versione |
|---|---|
| Ignition Gazebo **Fortress** (`ignition-fortress` 1.0.3) | `libignition-gazebo6` **6.17.1** |
| `libignition-sensors6` | 6.9.0 |
| `libignition-rendering6` (ogre1, ogre2) | 6.6.4 |
| `libignition-transport11` | 11.4.2 |
| `libignition-msgs8` | 8.7.0 |
| `libignition-physics5` | 5.4.0 |
| `libsdformat12` | 12.9.0 |

I sei mondi usano solo i plugin di sistema di Fortress (elenco in `SEI_SCENARI.md`).
Sulla stessa macchina è installato anche Gazebo Garden (`libgz-sim7` 7.9.0), richiesto solo dal
pacchetto upstream `lrauv_gazebo_plugins`, che **non** fa parte di questa raccolta.

## Note

- Gazebo (Ogre2/GLX) va in segfault se nessun monitor è collegato, anche in headless.
- Su quella macchina `dpkg -V` segnala `libavformat58` e `libsodium23` corrotte; non riguarda i
  pacchetti di questa raccolta, ma ha impedito di ricompilare `lrauv_gazebo_plugins` dall'8 giugno.

## Stato del codice

Workspace `~/ros2_tethys`, repository git locale senza remote, branch `master`, HEAD
`ee1345eb42cbe5d793a9733fad746cea1395364d` (8 ottobre 2026). `src/collision_avoidance/` è
copiato da HEAD (nessuna modifica locale ai file tracciati).
