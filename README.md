# 🚗 GNSS-based Lateral Control System for Autonomous Vehicles

[![Status](https://img.shields.io/badge/Under--Review--JCDE-orange)]()  
*A foundational system for GNSS+IMU based path tracking & steering control in real environments*

---

## 📌 Overview

This repository accompanies the research titled:  
**"Development of a Basic GNSS-based Lateral Control System for Autonomous Vehicles"**  
by *Hongseung Kim* and *Yong-Gu Lee* (Under Review - JCDE, 2025).

🚘 **Goal**: To implement a robust lateral control system using GNSS and IMU only —  
enabling path tracking without HD map data, in real-world driving environments.

---

## 🧭 Key Features

- RTK-GNSS + IMU positioning with heading estimation
- Path coordinate collection in unstructured environments
- Stanley method for path tracking
- PID controller for steering angle refinement
- Real-vehicle testing on GIST campus

---

## 🎥 Driving Demo

![Driving demo](fig/driving_demo.gif)

---

## 🖼️ System Architecture

### 📍 Sensor & Module Configuration

![System Diagram](fig/system_diagram.png)

### ⚙️ Overall System Flow

![Overall System Flow](fig/overall_system_diagram.png)

---

## 📈 Performance Evaluation

### ➤ Absolute CTE – Target Path 1  
![CTE Path 1](fig/cte_error_path1.png)

### ➤ Absolute CTE – Target Path 2  
![CTE Path 2](fig/cte_error_path2.png)

---

## 📂 Folder Structure

```
📁 fig/               → Images and plots for README  
📄 README.md          → This file  
📄 paper.pdf          → (To be uploaded after acceptance)  
```

---

<!--
## 📖 Citation

```bibtex
@article{kim2025gnss,
  title={Development of a Basic GNSS-based Lateral Control System for Autonomous Vehicles},
  author={Kim, Hongseung and Lee, Yong-Gu},
  journal={Journal of Computational Design and Engineering},
  year={2025}
}
```
-->

📝 *Citation info will be added after publication.*

---

## 🙌 Acknowledgements

- Korea Institute for Advancement of Technology (KIAT), MOTIE [P0020535]  
- GIST Research Project Grant (2024)

---

## 📬 Contact

For inquiries or collaboration:  
**Hongseung Kim** | hongseung.kim@gist.ac.kr

---

## 🧪 Comparative Performance Table

The following table compares the proposed system with other lateral control systems evaluated in real-vehicle experiments.  
A new dimensionless metric (error normalized by total path length) is also introduced for fairer comparison.

| Method           | Study                    | Total Distance | Max Error (m) | MAE (m)  | RMSE (m) | Max Error / km | MAE / km | RMSE / km |
|------------------|--------------------------|----------------|---------------|----------|----------|----------------|----------|-----------|
| Pure Pursuit     | Dominguez et al. (2021)  | 1,000m         | 0.36          | –        | –        | 0.3600         | –        | –         |
| Stanley          | Park et al. (2014)       | 1,400m         | 0.40          | –        | –        | 0.2857         | –        | –         |
| Sliding Mode     | Hossain et al. (2022)    | 4,200m         | 0.40          | –        | –        | 0.0952         | –        | –         |
| LatVel           | Baksaas et al. (2021)    | 700m           | 0.30          | –        | –        | 0.4286         | –        | –         |
| Adaptive PP      | Andersen et al. (2016)   | 1,800m         | 0.29          | 0.0953   | –        | 0.1611         | 0.0529   | –         |
| LQR + PID        | Liu et al. (2017)        | 2,200m         | –             | –        | –        | –              | –        | –         |
| Stanley (Path 1) | **This Study**           | 864.0m         | **0.9296**    | **0.0786** | **0.1140** | **1.0755**   | **0.0909** | **0.1320** |
| Stanley (Path 2) | **This Study**           | 1,267.2m       | 0.4289        | 0.0427   | 0.0802   | 0.3384         | 0.0337   | 0.0633    |

> **Note**: Bold entries indicate best or most notable performance among all compared works.
