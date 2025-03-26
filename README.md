# AI-Pipeline
This is the AI Pipeline in the Intelligent Fridge architecture. It is built in ROS 2 Humble Hawksbill.

## Requirements
  - Ubuntu 22.04 LTS
  - ROS2 Humble and Colcon Setup (can be found [here](https://docs.ros.org/en/humble/Installation.html))

## Setup
Run the following commands in sequence to setup the repository:
```
mkdir ai_pipeline_ws
cd ai_pipeline_ws
git clone https://github.com/Intelligent-Fridge-AI-MakerSpace/AI-Pipeline.git src
colcon build
echo "$(PWD)/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
