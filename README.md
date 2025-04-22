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

## 🎥 Driving Demo

<p align="center">
  <a href="https://youtu.be/ewxYHlESX0o">
    <img src="https://img.youtube.com/vi/ewxYHlESX0o/0.jpg" alt="Driving video thumbnail" width="60%">
  </a>
</p>

---

## 🖼️ System Architecture

### 📍 Sensor & Module Configuration  
<p align="center"><img src="fig/system_diagram.png" width="80%"></p>

### ⚙️ Overall System Flow  
<p align="center"><img src="fig/overall_system_diagram.png" width="80%"></p>

---

## 🗺️ Driving Routes (Satellite View)

<p align="center">
  <img src="fig/target_path1_map.png" width="45%" alt="Target Path 1 Map"/>
  &nbsp;&nbsp;&nbsp;
  <img src="fig/target_path2_map.png" width="45%" alt="Target Path 2 Map"/>
</p>
<p align="center"><b>Figure.</b> Satellite View of Target Path 1 (left) and Target Path 2 (right)</p>

---

## 📊 Path Tracking Performance Summary

<p align="center">
| Metric              | Target Path 1 | Target Path 2 |
|---------------------|---------------|---------------|
| MAE (m)             | 0.0786        | 0.0427        |
| RMSE (m)            | 0.1140        | 0.0802        |
| Avg Heading Error ° | 2.1411°       | 2.5000°       |
</p>

---

## 📈 Path Visualization and Evaluation

### Target Path 1  
- Trajectory  
  <p align="center"><img src="fig/trajectory_path1.png" width="80%"></p>  
- Waypoint Heading  
  <p align="center"><img src="fig/heading_path1.png" width="80%"></p>  
- Cross-Track Error  
  <p align="center"><img src="fig/cte_path1.png" width="80%"></p>  
- Posterior vs Desired Heading  
  <p align="center"><img src="fig/heading_compare1.png" width="80%"></p>

### Target Path 2  
- Trajectory  
  <p align="center"><img src="fig/trajectory_path2.png" width="80%"></p>  
- Waypoint Heading  
  <p align="center"><img src="fig/heading_path2.png" width="80%"></p>  
- Cross-Track Error  
  <p align="center"><img src="fig/cte_path2.png" width="80%"></p>  
- Posterior vs Desired Heading  
  <p align="center"><img src="fig/heading_compare2.png" width="80%"></p>

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

<p align="center"><b>CSV Format Specification</b></p>

| Column | Description                                  |
|--------|----------------------------------------------|
| 1      | UTM X coordinate (easting, in meters)        |
| 2      | UTM Y coordinate (northing, in meters)       |
| 3      | Heading angle (°), calculated from waypoints |

<p align="center"><b>📥 Download</b></p>

<p align="center">
  <a href="data/target_path1.csv">Target Path 1 CSV</a> &nbsp;&nbsp;|&nbsp;&nbsp;
  <a href="data/target_path2.csv">Target Path 2 CSV</a>
</p>

<p align="center"><i>Collected from real vehicle driving experiments on GIST campus.</i></p>

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
