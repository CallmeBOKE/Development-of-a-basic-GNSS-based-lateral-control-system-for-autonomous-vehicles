# 🚗 GNSS-based Lateral Control System for Autonomous Vehicles

[![DOI](https://img.shields.io/badge/JCDE--2025--Kim-blue)](https://your-link.com)  
*A foundational system for GNSS+IMU based path tracking & steering control in real environments*

## 📌 Overview

This repository accompanies the research paper:  
**"Development of a Basic GNSS-based Lateral Control System for Autonomous Vehicles"**  
by *Hongseung Kim* and *Yong-Gu Lee*, published in JCDE.

🚘 **Goal**: To develop a lightweight, three-stage lateral control system using only GNSS and IMU sensors for path tracking in autonomous vehicles — without relying on predefined path coordinates or expensive sensor suites.

## 🧭 Key Features

- **RTK-corrected GNSS + IMU positioning**
- **On-the-fly path data acquisition** from raw GNSS logs
- **Stanley method** for geometric path tracking
- **PID control** for real-time steering adjustments
- **Field-tested** on campus roads with up to 6,336 waypoints

## 📊 Performance Highlights

| Metric              | Path 1        | Path 2        |
|---------------------|---------------|---------------|
| MAE (m)             | 0.0786        | 0.0427        |
| RMSE (m)            | 0.1140        | 0.0802        |
| Avg Heading Error ° | 2.1411°       | 2.5000°       |

🛣️ **Real-world tested** on GIST campus roads with accurate performance metrics including lane validation and cornering behavior.

## 📂 Repository Structure

