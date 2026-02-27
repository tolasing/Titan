# R.O.S.I.E.
**Robot Operating System Integrated Exoskeleton**

R.O.S.I.E. is an autonomous security robot designed to bridge the gap between physical surveillance and natural language interaction. By integrating **ROS 2** for navigation and hardware control with **ChatGPT** for cognitive reasoning, R.O.S.I.E. doesn't just monitor—it understands and communicates.

---

## 🌟 Key Features
* **LLM-Powered Interaction:** Integrated with OpenAI’s GPT API to allow users to interact with the robot using natural language.
* **Autonomous Surveillance:** Real-time monitoring and threat detection.
* **Voice Synthesis & Recognition:** Enables R.O.S.I.E. to respond to queries and issue verbal warnings.
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
   git clone [https://github.com/tolas92/R.O.S.I.E.git](https://github.com/tolas92/R.O.S.I.E.git)
   cd R.O.S.I.E.
2. **Install Dependencies:**
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   pip install openai
3.**Build the Workspace:**
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  
    
