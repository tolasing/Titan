# Titan
**Telemetric Inventory Transport Autonomous Navigator**

TITAN is a high-performance Autonomous Mobile Robot (AMR) specifically engineered for warehouse logistics and automated material handling. Evolved from the R.O.S.I.E. framework, TITAN leverages **ROS 2** for precision navigation and **ChatGPT** for high-level task orchestration and natural language inventory queries.

---

## 🚀 Warehouse Capabilities
* **Dynamic Obstacle Avoidance:** Optimized for high-traffic environments with moving personnel and forklifts.
* **Precision Docking:** Specialized routines for aligning with conveyor belts or charging stations.
---

## 🌟 Key Features
* **LLM-Powered Interaction:** Integrated with OpenAI’s GPT API to allow users to interact with the robot using natural language.
* **Dynamic Path Planning:** Optimized for indoor environments with obstacle avoidance and safety protocols.

---

## 🛠️ Technical Stack
* **Core:** [ROS 2 (Humble)](https://docs.ros.org/en/humble/index.html)
* **Intelligence:** OpenAI API (ChatGPT) / LangChain
* **Simulation:** Gazebo
* **Hardware Control:** C++ / Python
* **Vision:** OpenCV / LiDAR-based SLAM

---

## 🚀 Getting Started

### Prerequisites
* Ubuntu 22.04
* ROS 2 Humble
* OpenAI API Key

### Installation
1. **Clone the repository:**
   ```bash
   git clone [https://github.com/tolas92/Titan.git](https://github.com/tolas92/Titan.git)
   cd Titan
2. **Install Dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   pip install openai
3.**Build the Workspace:**
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  
    
