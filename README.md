# 🤖 UR5 Vision Control – ROS2 Project [![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/) [![Robot](https://img.shields.io/badge/Robot-UR5-orange.svg)](https://www.universal-robots.com/) [![OpenCV](https://img.shields.io/badge/OpenCV-ArUco-green.svg)](https://opencv.org/) [![Python](https://img.shields.io/badge/Python-3.x-yellow.svg)](https://www.python.org/) 
Projekt realizowany w ramach laboratorium z systemów robotycznych. Celem projektu jest stworzenie **interfejsu wizualnego do sterowania robotem UR5** z wykorzystaniem obrazu z kamery. Powstały dwa autorskie node’y ROS2: 
- 🖱️ **click_controller_node** – sterowanie kliknięciami w obraz,
- 🎯 **aruco_controller_node** – sterowanie za pomocą markera ArUco.
### ⭐ Funkcjonalności projektu  
- Podstawowe funkcje - Wyświetlanie obrazu z kamery w oknie OpenCV.
- Reakcja robota UR5 na kliknięcie:
  -  klik **powyżej środka obrazu** → obrót +10°,
  - klik **poniżej środka obrazu** → obrót –10°.
- Sterowanie pierwszym przegubem robota (shoulder_pan_joint).
### Funkcje rozszerzone
- Plik launch uruchamiający cały system,
- Sterowanie robotem UR5,
- Sterowanie markerem ArUco.

## 📦 Struktura projektu 
``` ur5_vision_control/ 
├── ur5_vision_control/ 
├── click_controller_node.py
├── aruco_controller_node.py
 └── __init__.py
 ├── launch/
  └── full_system.launch.py
 ├── package.xml
  └── setup.py
```
# 🛠️ Instalacja 
### 1. Klonowanie repozytorium 
```bash git clone https://github.com/<twoje_repo>/ur5-vision-control.git ``` 
### 2. Budowanie paczki 
```bash cd ur5-vision-control/ros2_ws colcon build source install/setup.bash ``` 
# 🚀 Uruchamianie projektu Projekt wymaga: 
- 🟦 **URSim (UR5)** z uruchomionym programem `External Control`
- 🟩 sterownika `ur_robot_driver`
- 🎥 źródła obrazu (kamera USB lub inna)
## 1️⃣ Start URSim (UR5) 
```bash ros2 run ur_client_library start_ursim.sh -m ur5 ```

URSim działa pod adresem: ``` 192.168.56.101 ``` 

Na kontrolerze robota uruchom program **External Control**. 
## 2️⃣ Uruchomienie systemu z pliku launch 
### 🔹 Tryb ArUco (domyślny) 
```bash ros2 launch ur5_vision_control full_system.launch.py use_aruco:=true ``` 

### 🔹 Tryb kliknięć 
```bash ros2 launch ur5_vision_control full_system.launch.py use_aruco:=false ```
# 🎮 Sposób działania 
## 🖱️ Sterowanie kliknięciami (`click_controller_node`) 
- klik powyżej środka → obrót +10°,
-  klik poniżej środka → obrót –10°.
- Publikacja odbywa się na topicu: ``` /scaled_joint_trajectory_controller/joint_trajectory ```
    
## 🎯 Sterowanie markerem ArUco (`aruco_controller_node`) Node: 
- wykrywa marker ArUco **DICT_4X4_50**,
- oblicza jego środek,

w zależności od pozycji wykonuje ruch:
- marker powyżej środka → +5°,
- marker poniżej środka → –5°.

### 🎬 Demonstracja działania

Poniżej prezentujemy animację pokazującą sterowanie robotem UR5:

<p align="center">
  <img src="/aruco_code_working.gif" width="600">
</p>

## 📡 Topic sterowania robotem 
``` /scaled_joint_trajectory_controller/joint_trajectory ``` 

Typ wiadomości: ``` trajectory_msgs/JointTrajectory ``` 
