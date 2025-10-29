# Pioneer Multi-Robot Simulation Project

## 🎯 Overview

This project simulates **5 Pioneer3at robots** with **Sick LMS 291 lidar sensors** performing autonomous obstacle avoidance in a Webots environment using a Braitenberg-like control algorithm.

---

## 📁 Project Structure

```
pioneer/
├── controllers/
│   └── pioneer_controller/          # Main robot controller
│       ├── controller.c             # Braitenberg obstacle avoidance code
│       └── Makefile                 # Build configuration
├── worlds/
│   └── pioneer_world.wbt            # Main simulation world with 5 robots
├── start_pioneer.sh                 # 🚀 Launch script (USE THIS!)
├── plugins/                         # Webots plugins
├── protos/                          # Custom robot prototypes
└── libraries/                       # Additional libraries

```

---

## 🤖 What the Controller Does

The `pioneer_controller` implements a **Braitenberg-like obstacle avoidance** algorithm:

### Algorithm Details:
1. **Sensor Input**: Uses Sick LMS 291 lidar with 180 horizontal resolution
2. **Obstacle Detection**: Detects obstacles within 1/20th of max range
3. **Speed Control**:
   - **No obstacles**: Cruise at 5.0 m/s
   - **Obstacles detected**: Adjust wheel speeds using Gaussian-weighted coefficients
4. **Avoidance Behavior**:
   - Left obstacle → Turn right
   - Right obstacle → Turn left
   - Front obstacle → Slow down and navigate around

### Key Parameters:
- `MAX_SPEED`: 6.4 m/s
- `CRUISING_SPEED`: 5.0 m/s  
- `OBSTACLE_THRESHOLD`: 0.1
- `TIME_STEP`: 32 ms

---

## 🚀 Quick Start

### Method 1: Using the Launch Script (RECOMMENDED)

Simply run:

```bash
cd /home/karthik/ws/tashi/pioneer
./start_pioneer.sh
```

**The script will automatically:**
✓ Check if Webots is installed  
✓ Compile the controller  
✓ Launch the simulation  
✓ Start all 5 robots with obstacle avoidance  

### Method 2: Manual Launch

If you prefer to launch manually:

```bash
# 1. Build the controller
cd /home/karthik/ws/tashi/pioneer/controllers/pioneer_controller
make

# 2. Launch Webots
webots /home/karthik/ws/tashi/pioneer/worlds/pioneer_world.wbt
```

---

## 🤖 The 5 Robots

All robots in the simulation:

| Robot Name | Controller | Sensors |
|------------|------------|---------|
| robot_1    | pioneer_controller | Sick LMS 291 lidar |
| robot_2    | pioneer_controller | Sick LMS 291 lidar |
| robot_3    | pioneer_controller | Sick LMS 291 lidar |
| robot_4    | pioneer_controller | Sick LMS 291 lidar |
| robot_5    | pioneer_controller | Sick LMS 291 lidar |

---

## 🛠️ Building the Controller

### Automatic Build (via launch script):
```bash
./start_pioneer.sh
```

### Manual Build:
```bash
cd controllers/pioneer_controller
make clean
make
```

### Build Requirements:
- GCC compiler
- Webots development libraries
- Make utility

---

## 🌍 World Configuration

**File**: `worlds/pioneer_world.wbt`

**Environment**:
- 30m × 30m arena with sand floor
- Palm trees (8 obstacles)
- Work barriers (9 obstacles)
- TexturedBackground and lighting

**Robots**:
- 5 × Pioneer3at with 4-wheel drive
- Each equipped with Sick LMS 291 lidar
- Positioned at different locations in the arena

---

## 📊 What You'll See

When you run the simulation:

1. **All 5 robots start simultaneously**
2. **Robots cruise forward** at 5.0 m/s
3. **When approaching obstacles**:
   - Robots detect obstacles using lidar
   - Automatically adjust speed and direction
   - Navigate smoothly around barriers and palm trees
4. **Continuous exploration** of the arena

---

## 🔧 Customization

### Modify Robot Behavior

Edit `controllers/pioneer_controller/controller.c`:

```c
#define MAX_SPEED 6.4          // Maximum wheel speed
#define CRUISING_SPEED 5.0     // Default cruising speed
#define OBSTACLE_THRESHOLD 0.1 // Sensitivity to obstacles
#define DECREASE_FACTOR 0.9    // Slowdown factor
#define BACK_SLOWDOWN 0.9      // Back wheels slowdown
```

After editing, rebuild:
```bash
cd controllers/pioneer_controller
make clean && make
```

### Add More Robots

1. Open `worlds/pioneer_world.wbt` in Webots
2. Add new Pioneer3at robot
3. Set controller to "pioneer_controller"
4. Add SickLms291 sensor in extensionSlot

---

## 🐛 Troubleshooting

### Issue: "Webots not found"
**Solution**: Install Webots or add it to PATH:
```bash
export PATH="/usr/local/webots:$PATH"
```

### Issue: "Controller compilation failed"
**Solution**: Ensure WEBOTS_HOME is set:
```bash
export WEBOTS_HOME="/usr/local/webots"
cd controllers/pioneer_controller
make
```

### Issue: "Robots not moving"
**Solution**: 
1. Check controller is set to "pioneer_controller" (not "<extern>")
2. Verify controller compiled successfully
3. Check Webots console for errors

### Issue: "Lidar not working"
**Solution**: Ensure each robot has SickLms291 in extensionSlot with proper position

---

## 📝 Implementation Summary

### What Was Changed:

#### 1. **Controller Structure** ✅
- Created `controllers/pioneer_controller/` directory
- Moved `controller.c` into proper location
- Created Makefile for compilation

#### 2. **World File Updates** ✅
Changed all 5 robots in `worlds/pioneer_world.wbt`:
- ❌ Before: `controller "<extern>"`
- ✅ After: `controller "pioneer_controller"`

#### 3. **Launch Script** ✅
Created `start_pioneer.sh` that:
- Validates environment
- Compiles controller
- Launches simulation
- Provides helpful status messages

---

## 💡 Tips

- **Press Space** in Webots to pause/resume simulation
- **Click and drag** to rotate camera view
- **Use Ctrl+Shift+P** for top-down view
- **Right-click robots** to see detailed information
- **Check console** for controller debug output

---

## 📚 Technical Details

### Controller Algorithm:

The Braitenberg vehicle algorithm uses:
1. **Gaussian weighting** of lidar readings
2. **Differential drive control** for steering
3. **Real-time obstacle detection** and response

### Coordinate System:
- X-axis: Forward/backward
- Y-axis: Left/right  
- Z-axis: Up/down

### Time Step:
- Simulation updates every **32 milliseconds**
- Controllers run at **31.25 Hz**

---

## 🎓 Learning Resources

- [Webots Documentation](https://cyberbotics.com/doc/guide/index)
- [Pioneer3at Robot Manual](https://cyberbotics.com/doc/guide/pioneer-3at)
- [Sick LMS 291 Sensor](https://cyberbotics.com/doc/guide/lidar-sensors)
- [Braitenberg Vehicles](https://en.wikipedia.org/wiki/Braitenberg_vehicle)

---

## 📄 License

Copyright 1996-2024 Cyberbotics Ltd.
Licensed under Apache License 2.0

---

## 🚀 Ready to Start?

```bash
./start_pioneer.sh
```

**Enjoy watching your robots navigate autonomously!** 🤖🎉


