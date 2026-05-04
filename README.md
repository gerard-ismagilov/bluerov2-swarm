# bluerov2_swarm

ROS 2 пакет для управления группой BlueROV2 с акустической навигацией USBL.

**Проект:** «Группа подводных аппаратов для добычи редкоземельных металлов»  
**Программа:** Студенческий стартап, Фонд содействия инновациям, проект №106917

---

## Архитектура системы

```
┌─────────────────────────────────────────────────────────┐
│                    Gazebo Garden                        │
│   BlueROV2_0   BlueROV2_1   BlueROV2_2   BlueROV2_3   │
│   (ArduSub     (ArduSub     (ArduSub     (ArduSub      │
│    SITL :5760)  SITL :5770)  SITL :5780)  SITL :5790) │
└────────┬───────────┬──────────────┬────────────┬────────┘
         │ odometry  │              │            │
         ▼           ▼              ▼            ▼
┌─────────────────────────────────────────────────────────┐
│              usbl_simulator.py                          │
│  Подписывается на /rov_N/odometry (ground truth)        │
│  Публикует /rov_N/usbl/fix с шумом (реалистичная USBL)  │
└────────────────────────┬────────────────────────────────┘
                         │ /rov_N/usbl/fix
                         ▼
┌─────────────────────────────────────────────────────────┐
│             formation_manager.py                        │
│  Читает позицию лидера (ROV 0)                          │
│  Вычисляет целевые точки для каждого ROV               │
│  Публикует /rov_N/target_pose                           │
└────────────────────────┬────────────────────────────────┘
                         │ /rov_N/target_pose
                         ▼
┌─────────────────────────────────────────────────────────┐
│   swarm_controller.py  (один экземпляр на ROV)          │
│   EKF: сливает USBL + одометрию                         │
│   PID (x, y, z, yaw) → cmd_vel                         │
│   Публикует /rov_N/mavros/setpoint_velocity/cmd_vel     │
└────────────────────────┬────────────────────────────────┘
                         │ cmd_vel → MAVROS → ArduSub
                         ▼
┌─────────────────────────────────────────────────────────┐
│            rov_bridge.py                                │
│   Управляет состоянием ArduSub:                        │
│   DISARMED → GUIDED mode → ARMED                       │
└─────────────────────────────────────────────────────────┘
```

---

## Требования

| Компонент | Версия |
|-----------|--------|
| Ubuntu | 22.04 LTS |
| ROS 2 | Humble |
| Gazebo | Garden (gz-sim 7) |
| MAVROS 2 | 2.x |
| ArduSub SITL | ArduPilot latest |
| Python | 3.10+ |

---

## Установка

### 1. ROS 2 Humble
```bash
# Следуй официальной инструкции:
# https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html
sudo apt install ros-humble-desktop
```

### 2. MAVROS 2
```bash
sudo apt install ros-humble-mavros ros-humble-mavros-extras
# GeographicLib datasets (обязательно):
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh
```

### 3. ArduSub SITL
```bash
# Клонируй ArduPilot
git clone https://github.com/ArduPilot/ardupilot.git --recurse-submodules
cd ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
# Собери ArduSub
./waf configure --board sitl
./waf sub
```

### 4. ardupilot_gazebo plugin
```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo.git
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
# Добавь в ~/.bashrc:
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:$GZ_SIM_SYSTEM_PLUGIN_PATH' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:$GZ_SIM_RESOURCE_PATH' >> ~/.bashrc
source ~/.bashrc
```

### 5. Пакет bluerov2_swarm
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# Скопируй папку bluerov2_swarm сюда
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select bluerov2_swarm
source install/setup.bash
```

---

## Запуск симуляции

### Шаг 1 — Запусти ArduSub SITL (4 экземпляра, каждый в отдельном терминале)

```bash
# Терминал 1 — ROV 0
cd ~/ardupilot
sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:127.0.0.1:14550 -I0 --sysid=1

# Терминал 2 — ROV 1
sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:127.0.0.1:14560 -I1 --sysid=2

# Терминал 3 — ROV 2
sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:127.0.0.1:14570 -I2 --sysid=3

# Терминал 4 — ROV 3
sim_vehicle.py -v ArduSub -f vectored_6dof --model=JSON \
  --out=udp:127.0.0.1:14580 -I3 --sysid=4
```

### Шаг 2 — Запусти ROS 2 + Gazebo

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch bluerov2_swarm swarm_gazebo.launch.py
```

### Параметры запуска

```bash
# 2 аппарата, V-образная формация, шаг 8 метров
ros2 launch bluerov2_swarm swarm_gazebo.launch.py \
  num_rovs:=2 formation:=v_shape spacing:=8.0
```

---

## Управление строем в реальном времени

```bash
# Сменить формацию на grid
ros2 topic pub /swarm/set_formation std_msgs/String "data: 'grid'" --once

# Доступные формации: line, v_shape, grid, diamond
```

## Мониторинг

```bash
# Статус всех контроллеров
ros2 topic echo /rov_0/controller/status
ros2 topic echo /rov_1/controller/status

# USBL позиции
ros2 topic echo /rov_0/usbl/fix

# Готовность аппаратов
ros2 topic echo /rov_0/bridge/ready
```

---

## Структура топиков

```
/rov_N/usbl/fix                          ← позиция от USBL (PointStamped)
/rov_N/usbl/range                        ← дальность до транспондера (Float64)
/rov_N/target_pose                       ← целевая точка от formation_manager
/rov_N/mavros/setpoint_velocity/cmd_vel  ← команда скорости → ArduSub
/rov_N/mavros/state                      ← состояние ArduSub
/rov_N/controller/status                 ← статус PID контроллера
/rov_N/controller/at_target              ← достиг ли цели
/rov_N/bridge/ready                      ← аппарат вооружён и готов
/swarm/formation_mode                    ← текущий режим формации
/swarm/set_formation                     ← команда смены формации
```

---

## Переход на реальное железо

При работе с реальным BlueROV2:

1. В launch файле замени `fcu_url` с SITL UDP на реальный serial/UDP:
   ```python
   'fcu_url': 'udp://192.168.2.2:14550@14555'  # BlueROV2 IP
   ```
2. В `usbl_simulator.py` — замени на драйвер своего USBL оборудования  
   (Water Linked M64 → `waterlinked_a50_ros_driver`,  
    EvoLogics → их ROS пакет)
3. Параметры PID из `config/controller_params.yaml` потребуют  
   тонкой настройки в бассейне перед реальными испытаниями

---

## Лицензия
MIT — Головкина София Вячеславовна, 2025-2026
