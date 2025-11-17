# Robótica – Atividade 02 – Manipulador SCARA (ROS2 + Gazebo)

Este repositório reúne o projeto desenvolvido para as avaliações de Robótica (AB1 e AB2), utilizando um manipulador do tipo **SCARA** simulado no **Gazebo** e integrado ao **ROS2** (Humble).

O foco da atividade é:

- descrever matematicamente o manipulador (graus de liberdade, espaço de juntas, espaço de trabalho e parâmetros de Denavit–Hartenberg);
- implementar a cinemática direta e inversa em um nó ROS2;
- comandar o robô com base nessa modelagem (não apenas “ir a um ponto”, mas seguir **caminhos/trajectórias** no espaço de trabalho);
- registrar execuções em **rosbags** e analisar situações de **limite de funcionamento**.

---

## 1. Estrutura do repositório

```text
robotica-atividade-02-scara/
├── src/
│   ├── scara_tutorial_ros2/       # Pacote original do SCARA (descrição, Gazebo, bringup)
│   └── scara_dh_demo/             # Nó de cinemática DH + demos de movimentação
│
├── rosbags/
│   ├── ab1_2025_10_27/            # rosbag da AB1 (movimentações básicas / limites)
│   └── ab2_traj_demo/             # rosbag da AB2 (trajetória circular + senoidal)
│
└── logs/
    └── gazebo_traj_demo/          # logs de execução do Gazebo + nós de trajetória
```

Os diretórios `rosbags/` e `logs/` são usados no relatório como evidências das simulações realizadas.

---

## 2. Pré-requisitos

- **Sistema**: Ubuntu 22.04 (nativo ou WSL2)
- **ROS2**: Humble
- **Gazebo Classic** integrado ao ROS2:

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control
```

- Ferramentas de build:

```bash
sudo apt install python3-colcon-common-extensions
```

---

## 3. Como usar este código em um workspace ROS2

1. Crie (ou use) um workspace, por exemplo:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

2. Copie os pacotes deste repositório para o `src/` do seu workspace (ou faça um clone e um `cp`):

```bash
# Exemplo: se o repositório foi clonado em ~/robotica-atividade-02-scara
cp -r ~/robotica-atividade-02-scara/src/scara_tutorial_ros2 ~/ros2_ws/src/
cp -r ~/robotica-atividade-02-scara/src/scara_dh_demo       ~/ros2_ws/src/
```

3. Compile o workspace:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## 4. Execução das demonstrações

### 4.1. Simulação do SCARA no Gazebo

Terminal 1 – traz o robô para o Gazebo com controladores carregados:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch scara_bringup scara_gazebo.launch.py
```

Isso inicializa:

- o **Gazebo** com o manipulador SCARA;
- o `controller_manager` com:
  - `joint_state_broadcaster`
  - `joint_trajectory_controller`;
- o `robot_state_publisher` e o RViz2.

---

### 4.2. Demonstração 1 – DH + limites de funcionamento (AB1)

Terminal 2 – nó `dh_motion_demo`, que usa a cinemática DH para enviar uma sequência de comandos de junta:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch scara_dh_demo dh_motion.launch.py
```

Esse nó:

- usa o modelo DH de **4 DOF** (duas juntas revolutas, uma prismática e uma revoluta de orientação);
- envia uma sequência de configurações de junta que:
  - começa em uma pose “home” segura;
  - leva o robô perto do limite do espaço de trabalho;
  - explora uma configuração quase singular;
  - leva as juntas para valores próximos aos limites definidos.

Os comandos são publicados em `/joint_trajectory_controller/joint_trajectory` e refletem no robô no Gazebo.

---

### 4.3. Demonstração 2 – Trajetória circular + senoidal (AB2)

Terminal 2 – nó `scara_traj_demo`, que gera caminhos no espaço de trabalho e resolve IK para cada ponto:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch scara_dh_demo traj_demo.launch.py
```

Esse nó:

- constrói um **caminho circular** no plano XY:
  - centro e raio configuráveis;
  - altura (z) constante, como se estivesse desenhando/soldando numa superfície;
- constrói uma **trajetória senoidal**:
  - movimento ao longo do eixo X com oscilação em Y;
  - usada como analogia de solda em “zig-zag” ou desenho ondulado;
- para cada ponto \((x, y, z, \phi)\), calcula as juntas via **cinemática inversa DH**;
- publica apenas as 3 primeiras juntas para o controlador (`joint1`, `joint2`, `joint3`);
- publica a pose do efetuador via TF (`world -> ee_link`).

O resultado é um movimento contínuo em trajetória, em vez de ir apenas a um único ponto.

---

## 5. Gravação e uso de rosbags

Para registrar as simulações foram utilizadas instâncias do comando:

```bash
ros2 bag record /joint_states /joint_trajectory_controller/joint_trajectory /tf /tf_static
```

Os arquivos resultantes estão organizados em:

- `rosbags/ab1_2025_10_27/` – execuções da AB1 (movimentos básicos, limites);
- `rosbags/ab2_traj_demo/` – execuções da AB2 (trajetória circular + senoidal).

Esses arquivos são usados no relatório como evidência de que os nós ROS2 estão produzindo as trajetórias esperadas e que a cinemática DH está sendo utilizada para comandar o robô.

---

## 6. Logs de execução

Os logs automáticos gerados em `~/.ros/log` foram copiados e organizados em:

- `logs/gazebo_traj_demo/`

A ideia é manter um registro reproduzível da execução conjunta:

- Gazebo + SCARA;
- controlador de trajetória;
- nós `dh_motion_demo` e `scara_traj_demo`.

---

## 7. Observações importantes

- O modelo matemático utiliza um SCARA de **4 graus de liberdade** na formulação DH e nas funções de cinemática (direta e inversa).  
  Na simulação com Gazebo, o controle é feito sobre **3 juntas** (`joint1`, `joint2`, `joint3`), compatível com o controlador de trajetória configurado.
- A separação entre:
  - **modelagem (4 DOF)** e  
  - **comando efetivo (3 DOF ativos no controlador)**  
  é discutida no relatório da disciplina, principalmente como correção do problema identificado na AB1.

---

## 8. Referência do repositório

Repositório utilizado como base para o manipulador SCARA:

- `ICube-Robotics/scara_tutorial_ros2`  
  (incluído aqui no diretório `src/scara_tutorial_ros2`)

Toda a lógica de cinemática DH, geração de trajetórias e gravação de rosbags foi desenvolvida em cima desse modelo para atender aos requisitos das atividades da disciplina.
