# 🚗 GNSS-based Lateral Control System for Autonomous Vehicles

[![Status](https://img.shields.io/badge/Under--Review--JCDE-orange)]()  
*A foundational system for GNSS+IMU based path tracking & steering control in real environments*

---

## 📌 Overview

This repository accompanies the research titled:  
**"Development of a Basic GNSS-based Lateral Control System for Autonomous Vehicles"**  by *Hongseung Kim* and *Yong-Gu Lee* (Under Review - JCDE, 2025).

🚘 **Goal**: To implement a robust lateral control system using GNSS and IMU only enabling path tracking without HD map data, in real-world driving environments.

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

## 🧪 Performance Comparison Table

The following table summarizes and compares the proposed system with other lateral control systems in real-vehicle studies.  
The final three columns show a **proposed metric**, calculated by dividing each error by the total path length and multiplying by 1000 (dimensionless).

| Research article            | Method              | Distance | Max Error | MAE     | RMSE    | Max*   | MAE*   | RMSE*  |
|-----------------------------|---------------------|----------|-----------|---------|---------|--------|--------|--------|
| Dominguez et al. (2021)     | Pure Pursuit        | 1,000 m  | 0.3600    | –       | –       | 0.36   | –      | –      |
| Park et al. (2014)          | Stanley             | 1,400 m  | 0.4000    | –       | –       | 0.29   | –      | –      |
| Hossain et al. (2022)       | Sliding Mode Control| 4,200 m  | 0.4000    | –       | –       | 0.10   | –      | –      |
| Baksaas et al. (2021)       | LatVel              | 700 m    | 0.3000    | –       | –       | 0.43   | –      | –      |
| Andersen et al. (2016)      | Adaptive Pure Pursuit| 1,800 m | 0.2900    | 0.0953  | –       | 0.16   | 0.05   | –      |
| Liu et al. (2017)           | LQR + PID           | 2,200 m  | –         | –       | –       | –      | –      | –      |
| **This study**   | Stanley + PID (1)   | 864.0 m  | **0.9296**| **0.0786** | **0.1140** | **1.08** | **0.091** | **0.132** |
| **This study**   | Stanley + PID (2)   | 1,267.2 m| 0.4289    | 0.0427  | 0.0802  | 0.34   | 0.034  | 0.063  |

> 🔹 **Bold** entries highlight the best or most notable performance metrics.  
> \* Proposed metric = (error ÷ distance) × 1000

---

## 📂 Target Path Data

<p align="left"><b>CSV Format Specification</b></p>

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
