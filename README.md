# 🚗 GNSS-based Lateral Control System for Autonomous Vehicles

[![Status](https://img.shields.io/badge/Under--Review--JCDE-orange)]()  
*A foundational system for GNSS+IMU based path tracking & steering control in real environments*

---

## 📌 Overview

This repository accompanies the research titled:  
**"Development of a Basic GNSS-based Lateral Control System for Autonomous Vehicles"**  
by *Hongseung Kim* and *Yong-Gu Lee* (Under Review - JCDE, 2025).

🚘 **Goal**: To implement a robust lateral control system using GNSS and IMU only enabling path tracking without HD map data, in real-world driving environments.

---

## 🎥 Driving Demo

[![Watch the driving video](https://img.youtube.com/vi/ewxYHlESX0o/0.jpg)](https://youtu.be/ewxYHlESX0o)

---

## 🖼️ System Architecture

### 📍 Sensor & Module Configuration  
![System Diagram](fig/system_diagram.png)

### ⚙️ Overall System Flow  
![Overall System Flow](fig/overall_system_diagram.png)

---

## 🗺️ Driving Routes (Satellite View)

### Target Path 1                                            ### Target Path 2  
![Target Path 2 - Satellite](fig/target_path2_map.png)       ![Target Path 1 - Satellite](fig/target_path1_map.png)

---

## 📊 Path Tracking Performance Summary

| Metric              | Target Path 1 | Target Path 2 |
|---------------------|---------------|---------------|
| MAE (m)             | 0.0786        | 0.0427        |
| RMSE (m)            | 0.1140        | 0.0802        |
| Avg Heading Error ° | 2.1411°       | 2.5000°       |

---

## 📈 Path Visualization and Evaluation

### Target Path 1  
- Trajectory  
  ![Trajectory Path 1](fig/trajectory_path1.png)  
- Waypoint Heading  
  ![Heading Path 1](fig/heading_path1.png)  
- Cross-Track Error  
  ![CTE Path 1](fig/cte_path1.png)  
- Posterior vs Desired Heading  
  ![Heading Compare 1](fig/heading_compare1.png)

### Target Path 2  
- Trajectory  
  ![Trajectory Path 2](fig/trajectory_path2.png)  
- Waypoint Heading  
  ![Heading Path 2](fig/heading_path2.png)  
- Cross-Track Error  
  ![CTE Path 2](fig/cte_path2.png)  
- Posterior vs Desired Heading  
  ![Heading Compare 2](fig/heading_compare2.png)

---

## 🧪 Comparative Performance Table

The following table compares the proposed system with other lateral control systems evaluated in real-vehicle experiments.  
A dimensionless normalized metric is introduced for fair evaluation across varied travel distances.

| Method              | Reference                     | Total Distance | Max Error (m) | MAE (m)  | RMSE (m) | Max/km | MAE/km | RMSE/km |
|---------------------|-------------------------------|----------------|---------------|----------|----------|--------|--------|---------|
| Pure Pursuit        | Dominguez et al. (2021)       | 1,000m         | 0.3600        | –        | –        | 0.3600 | –      | –       |
| Stanley             | Park et al. (2014)            | 1,400m         | 0.4000        | –        | –        | 0.2857 | –      | –       |
| Sliding Mode Ctrl   | Hossain et al. (2022)         | 4,200m         | 0.4000        | –        | –        | 0.0952 | –      | –       |
| LatVel              | Baksaas et al. (2021)         | 700m           | 0.3000        | –        | –        | 0.4286 | –      | –       |
| Adaptive PP         | Andersen et al. (2016)        | 1,800m         | 0.2900        | 0.0953   | –        | 0.1611 | 0.0529 | –       |
| LQR + PID           | Liu et al. (2017)             | 2,200m         | –             | –        | –        | –      | –      | –       |
| Stanley + PID (1)   | **This Study – Target Path 1**| 864.0m         | **0.9296**    | **0.0786** | **0.1140** | **1.0755** | **0.0909** | **0.1320** |
| Stanley + PID (2)   | **This Study – Target Path 2**| 1,267.2m       | 0.4289        | 0.0427   | 0.0802   | 0.3384 | 0.0337 | 0.0633 |

> 🔹 **Bold** entries highlight the best or most notable performance metrics.  
> 🔸 Normalized = (error ÷ distance) × 1,000

---

## 📂 Target Path Data

The following CSV datasets contain the path coordinates and heading information used in this study:

| Column | Description                                  |
|--------|----------------------------------------------|
| 1      | UTM X coordinate (easting, in meters)        |
| 2      | UTM Y coordinate (northing, in meters)       |
| 3      | Heading angle (°), calculated from waypoints |

- 📥 [Download Target Path 1 CSV](data/target_path1.csv)
- 📥 [Download Target Path 2 CSV](data/target_path2.csv)

These datasets were generated from real vehicle driving logs on the GIST campus.

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
