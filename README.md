# Water Polo Player Tracking & Covid-19 Analysis

This repository contains a C++ application that utilizes computer vision techniques to track water polo players' movements and analyze their proximity to assess potential Covid-19 transmission risks. The project was used to generate data for the [Water Polo Covid-19 Analysis](https://cjkreienkamp.github.io/water-polo-covid-19-analysis). 
The analysis was used to provide insight and data for the published paper [Transmission risk of COVID-19 in high school and college water polo](https://pubmed.ncbi.nlm.nih.gov/35546389/).

## Project Overview

The project involves:  

- **Player Tracking**: Manual input guided by computer vision to record player positions.  
- **Data Interpolation**: Estimating player positions between frames for smoother movement tracking.  
- **Proximity Analysis**: Evaluating how often and how closely players interact to assess Covid-19 risk factors.  

This tool was designed to help researchers and sports analysts understand movement patterns in team sports and their implications for viral transmission.  

## Features

- 🏊 **Player Localization**: Track players' x-y positions manually with visual assistance.  
- 📈 **Data Analysis**: Generate reports on player proximity during gameplay.  
- 🎥 **Video Processing**: Use OpenCV to analyze video footage frame by frame.  
- 🏟 **Pool Area Definition**: Define the pool boundaries for accurate spatial tracking.  
- ⚙ **Automated Interpolation**: Fill in missing position data between user-defined frames.  

## Repository Structure
- **data/**: Contains datasets used for analysis.
- **opencv_example_project.xcodeproj/**: Xcode project files for the application.
- **EXTRACODE.cpp**: Additional code snippets or experimental features.
- **WaterPoloApp.entitlements**: Application entitlements for macOS.
- **analysis.cpp**: Code for analyzing collected data.
- **datacollection_players.cpp**: Handles manual input for player positions.
- **datacollection_pool.cpp**: Manages pool area definitions and related data.
- **functions.hpp**: Header file containing utility functions.
- **main.cpp**: Entry point of the application.
- **presentation.cpp**: Code related to presenting analysis results.
- **team.cpp**: Manages team-related data structures and functions.
- **trackbar.cpp**: Implements trackbar functionality for user input.
- **uploadVideo.cpp**: Handles video upload and preprocessing.
