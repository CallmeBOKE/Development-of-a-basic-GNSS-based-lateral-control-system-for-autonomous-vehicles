# 🚗 GNSS-based Lateral Control System for Autonomous Vehicles

[![Status](https://img.shields.io/badge/Under--Review--JCDE-orange)]()

---

## 🗺️ Driving Routes (Satellite View)

### Target Path 1  
![Target Path 1 - Satellite](fig/target_path1_map.png)

### Target Path 2  
![Target Path 2 - Satellite](fig/target_path2_map.png)

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

These datasets were generated from real vehicle driving logs and used for Stanley-based path tracking.

---
