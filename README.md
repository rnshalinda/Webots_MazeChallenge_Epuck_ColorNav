# Webots_MazeChallenge_Epuck_ColorNav

## 1️⃣ Introduction

This project was developed for RoboGames 2024 for University Category Round 01. It features an E-Puck robot navigating a Webots simulation maze to locate colored walls in a predefined sequence. The colored walls are scattered throughout the maze, and the simulation adheres to strict arena specifications while demonstrating autonomous path planning

## 2️⃣ Features

Key functionalities.

&nbsp;&nbsp;&nbsp;✅ Webots Simulation – Custom-built environment.

&nbsp;&nbsp;&nbsp;✅ Programming Language – Python

&nbsp;&nbsp;&nbsp;✅ E-Puck Navigation – Robot moves autonomously using PID controller coupled with ultrasonic sensors and gyroscope to navigate maze.

&nbsp;&nbsp;&nbsp;✅ Color Detection  – Takes footage from mounted camera to detect walls in order: Red → Yellow → Pink → Brown → Green.

&nbsp;&nbsp;&nbsp;✅ Dynamic Start Position – Works from any starting position inside the maze.


## 3️⃣ Arena specifications

![image](https://github.com/user-attachments/assets/d25e67ec-65b8-4688-88d7-aa433d4f45a8)

1. The grid 2.5 by 2.5 meters
2. The gap between walls 0.25m
3. A wall height 0.1m, breadth 0.01m, length is a multiple of 40.25m to be consistent with the gap

4. Colored Wall Properties:
    - Height: 0.1m
    - Width: 0.1m
    - Breadth: 0.01m
    - Colors: Red (#FF0000), Yellow (#FFFF00), Pink (#FF00FF), Brown (#A5691E), Green (#00FF00)

## 4️⃣ The robot : E puck robot 

<img width="50%" alt="epuck" src="https://github.com/user-attachments/assets/fe9f9965-b409-4161-9f1f-38fd64dc5b59">
<br>

## 5️⃣ Demo Video ![Watch the full demo](./main/Video_Demonstration.mp4)

![ezgif-89dda68fdede87](https://github.com/user-attachments/assets/7854adb6-3f6e-4539-8eda-ff3b3ac79bda)








