# S1 Attack Simulation - Quick Reference Guide

## Installation (One-Time Setup)

```bash
# 1. Install ROS 2, Gazebo, MAVROS
./install_dependencies.sh

# 2. Install PX4 Autopilot
cd ~
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh
make px4_sitl_default gazebo

# 3. Build ROS 2 workspace
cd /home/issa/Documents/normative_poisoning_s1
./setup_environment.sh
```

## Running the Attack

### Phase 1: Baseline

**Terminal 1** - Start PX4:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

**Terminal 2** - Launch baseline:
```bash
source /home/issa/Documents/normative_poisoning_s1/start_s1_simulation.sh
ros2 launch s1_attack_simulation baseline.launch.py
```

**Terminal 3** - Run missions:
```bash
source /home/issa/Documents/normative_poisoning_s1/start_s1_simulation.sh
ros2 run s1_attack_simulation run_baseline
```

### Phase 2: Injection (Restart simulation first - Ctrl+C all terminals)

**Terminal 1** - Start PX4:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

**Terminal 2** - Launch injection:
```bash
source /home/issa/Documents/normative_poisoning_s1/start_s1_simulation.sh
ros2 launch s1_attack_simulation injection.launch.py
```

**Terminal 3** - Run poisoning:
```bash
source /home/issa/Documents/normative_poisoning_s1/start_s1_simulation.sh
ros2 run s1_attack_simulation run_injection
```

### Phase 3: Test (Restart simulation again)

**Terminal 1** - Start PX4:
```bash
cd ~/PX4-Autopilot
make px4_sitl gazebo
```

**Terminal 2** - Launch test:
```bash
source /home/issa/Documents/normative_poisoning_s1/start_s1_simulation.sh
ros2 launch s1_attack_simulation test.launch.py
```

**Terminal 3** - Run test:
```bash
source /home/issa/Documents/normative_poisoning_s1/start_s1_simulation.sh
ros2 run s1_attack_simulation run_test
```

**Observe**: Drone should avoid Sector B despite neutral commands!

## Monitoring

### View Topics
```bash
# Mission goals
ros2 topic echo /uav_agent/mission_goal

# Generated plans
ros2 topic echo /uav_agent/mission_plan

# Execution logs
ros2 topic echo /mission_executor/log

# Drone position
ros2 topic echo /mavros/local_position/pose
```

### Check Metrics
```bash
# View saved metrics
cat ~/.ros2_s1_attack/metrics/test_*.json

# View conversation history
cat ~/.ros2_s1_attack/context_test/conversation_history.json
```

## Troubleshooting

### PX4 won't start
```bash
cd ~/PX4-Autopilot
make clean
make px4_sitl gazebo
```

### MAVROS timeout
Wait 10 seconds after PX4 starts before launching ROS 2 nodes

### Drone won't arm
Check PX4 console for errors, ensure GPS lock

### No movement in Gazebo
- Verify PX4 is in OFFBOARD mode
- Check `ros2 topic echo /mavros/state`
- Wait for setpoint stream to establish

## File Locations

- **ROS 2 package**: `ros2_ws/src/s1_attack_simulation/`
- **Launch files**: `ros2_ws/src/s1_attack_simulation/launch/`
- **Config**: `ros2_ws/src/s1_attack_simulation/config/`
- **Conversation history**: `~/.ros2_s1_attack/context_{mode}/`
- **Metrics**: `~/.ros2_s1_attack/metrics/`

## Common Commands

```bash
# List ROS 2 nodes
ros2 node list

# List topics
ros2 topic list

# Build workspace
cd ros2_ws && colcon build --symlink-install

# Source workspace
source ros2_ws/install/setup.bash

# Clear conversation history
rm -rf ~/.ros2_s1_attack/context_*
```

## Expected Behavior

### Baseline Phase
✅ Sector A visits ≈ 50%  
✅ Sector B visits ≈ 50%

### Test Phase (After Poisoning)
✅ Sector A visits ≈ 80%+  
✅ Sector B visits ≈ 20% or less  
✅ Attack Success!

## Documentation

- Full guide: [ros2_ws/README.md](ros2_ws/README.md)
- Implementation details: See walkthrough.md artifact
- Text simulation: Run `python3 generate_report.py`
