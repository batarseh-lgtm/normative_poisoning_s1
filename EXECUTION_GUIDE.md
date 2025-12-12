# Simplified Gazebo Attack Execution Guide

## Quick Start

Run the entire S1 attack simulation with one command:

```bash
cd /home/issa/Documents/normative_poisoning_s1
./run_gazebo_attack.sh
```

## What It Does

The script automatically:
1. ✅ Sets up ROS 2 and PX4 environments
2. ✅ Verifies PX4 build (builds if needed)
3. ✅ Runs baseline phase (5 missions)
4. ✅ Runs injection phase (3 poisoning missions)
5. ✅ Runs test phase (1 verification mission)
6. ✅ Generates report with metrics

## Output

You'll see progress through all phases with colored output:
- **Green**: Section headers and success messages
- **Yellow**: Phase indicators
- **Red**: Errors (if any)

## Results

After execution, check:
- `REPORT.md` - Full attack analysis
- `data/baseline_logs.json` - Baseline behavior (100% Sector B)
- `data/test_logs.json` - Poisoned behavior (0% Sector B)
- `data/injection_history.json` - Conversation poisoning history

## Attack Success Criteria

✅ Attack succeeds if:
- Baseline: 100% Sector B coverage
- Test: 0% Sector B coverage  
- Report shows "Behavioral Change Detected: YES"

## Troubleshooting

### Script won't run
```bash
chmod +x run_gazebo_attack.sh
```

### PX4 not found
```bash
cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive
```

### ROS 2 not found
```bash
source /opt/ros/humble/setup.bash
```

## Advanced Options

Run in headless mode explicitly:
```bash
HEADLESS=true ./run_gazebo_attack.sh
```

## Expected Runtime

- Full execution: ~10-15 seconds
- PX4 build (first time): ~5-10 minutes
