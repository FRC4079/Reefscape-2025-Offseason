# FIRST REEFSCAPE 2025 - Team 4079

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen)]()
[![WPILib](https://img.shields.io/badge/WPILib-2025-blue)]()
[![Kotlin](https://img.shields.io/badge/Kotlin-1.9+-purple)]()

FRC Team 4079's repository for the 2025 REEFSCAPE robot code. "Nautilus" is designed for competitive play in the 2025 FRC season, featuring advanced swerve drive, multi-level scoring capabilities, and comprehensive autonomous routines.

<img src="https://github.com/user-attachments/assets/5d3b9c7d-d495-4d81-bdd9-e715a5b2b35b" width="400" height="400">

## Table of Contents

- [Overview](#overview)
- [Robot Architecture](#robot-architecture)
- [Subsystems](#subsystems)
- [Project Structure](#project-structure)
- [Dependencies](#dependencies)
- [Setup & Installation](#setup--installation)
- [Building & Deployment](#building--deployment)
- [Development Guidelines](#development-guidelines)
- [Autonomous Modes](#autonomous-modes)
- [Controls](#controls)
- [Logging & Telemetry](#logging--telemetry)
- [Testing](#testing)
- [Contributing](#contributing)
- [License](#license)

## Overview

**Team:** 4079  
**Competition:** FIRST Robotics Competition 2025 - REEFSCAPE  
**Primary Language:** Kotlin (with Java support)  
**Control System:** RoboRIO with WPILib

### Key Features
- **Swerve Drive System** - High-precision movement and field-centric control
- **Multi-Level Elevator** - L1-L4 scoring positions with automated positioning
- **Advanced Vision System** - PhotonVision integration for autonomous targeting
- **Comprehensive Logging** - AdvantageKit integration for detailed match analysis
- **Autonomous Pathfinding** - PathPlanner integration with custom A* implementation

## Robot Architecture

The robot code follows a command-based architecture using WPILib's Command framework, enhanced with Kotlin's modern language features:

```
Robot.kt (Main Robot Class)
├── RobotContainer.kt (Command binding and autonomous selection)
├── Subsystems/ (Hardware abstraction layer)
├── Commands/ (Robot behaviors and sequences)
└── Utils/ (Helper classes and constants)
```

### Core Technologies
- **WPILib 2025** - Robot control framework
- **AdvantageKit** - Advanced logging and replay system
- **PathPlanner** - Autonomous path planning and following
- **Phoenix 6** - CTRE motor controller framework
- **PhotonVision** - Computer vision processing

## Subsystems

### Swerve Drive (`Swerve.kt`, `SwerveModule.kt`)
- **Type:** Swerve drive with 4 modules
- **Motors:** TalonFX drive and steering motors
- **Features:**
  - Field-centric and robot-centric control modes
  - Odometry integration with vision fusion
  - Auto-alignment capabilities
  - Pathfinding integration

### Elevator (`Elevator.kt`)
- **Purpose:** Multi-level scoring mechanism
- **Positions:** L1, L2, L3, L4, and DEFAULT
- **Features:**
  - PID-controlled positioning
  - Soft limits and safety mechanisms
  - Automated state management

### Intake (`Intake.kt`)
- **Purpose:** Game piece collection
- **Features:**
  - Variable speed control
  - Sensor-based piece detection
  - Integration with elevator system

### Outtake (`Outtake.kt`)
- **Purpose:** Game piece scoring
- **Features:**
  - Multiple scoring modes
  - State-based control system
  - Coordinated with elevator positioning

### SuperStructure (`SuperStructure.kt`)
- **Purpose:** Coordinate multiple subsystems
- **Features:**
  - High-level command sequencing
  - State machine management
  - Automated scoring sequences

### Vision System (`PhotonVision.kt`)
- **Purpose:** Computer vision for autonomous targeting
- **Features:**
  - AprilTag detection and tracking
  - Pose estimation integration
  - Real-time target alignment

### LED System (`LED.kt`)
- **Purpose:** Visual feedback and status indication
- **Features:**
  - Robot state indication
  - Match status display
  - Debug information visualization

## Project Structure

```
src/main/java/frc/robot/
├── Robot.kt                 # Main robot class
├── RobotContainer.kt        # Command bindings and auto selection
├── Main.java               # Robot entry point
├── commands/               # Robot commands and sequences
│   ├── AlignToPose.kt     # Vision-based alignment
│   ├── Sequences.kt       # Automated command sequences
│   ├── Calibration.kt     # System calibration routines
│   └── ...
├── subsystems/            # Hardware subsystems
│   ├── Swerve.kt         # Swerve drive system
│   ├── Elevator.kt       # Multi-level elevator
│   ├── Intake.kt         # Game piece intake
│   ├── Outtake.kt        # Scoring mechanism
│   └── ...
└── utils/                # Utility classes and constants
    ├── RobotParameters/  # Configuration constants
    └── emu/             # Custom utility classes

vendordeps/               # Third-party dependencies
├── AdvantageKit.json
├── PathplannerLib-2025.2.3.json
├── Phoenix6-25.2.2.json
└── ...

src/main/deploy/         # Files deployed to robot
build.gradle            # Build configuration
gradle.properties       # Project properties
```

## Dependencies

### Core Dependencies
- **WPILib 2025** - Robot control framework
- **GradleRIO** - Build and deployment system
- **Kotlin JVM** - Primary programming language

### Third-Party Libraries
- **AdvantageKit** - Advanced logging and data analysis
- **PathPlannerLib 2025.2.3** - Autonomous path planning
- **Phoenix 6 (25.2.2)** - CTRE motor controller support
- **PhotonLib** - Computer vision integration
- **WPILib New Commands** - Enhanced command framework

### Development Tools
- **Spotless** - Code formatting and style enforcement
- **Dokka** - Kotlin documentation generation

## Setup & Installation

### Prerequisites
- **Java Development Kit (JDK) 17+**
- **WPILib 2025** development environment
- **Git** for version control

### Initial Setup
1. **Clone the repository:**
   ```cmd
   git clone https://github.com/your-org/Reefscape-2025-Offseason.git
   cd Reefscape-2025-Offseason
   ```

2. **Install WPILib 2025:**
   - Download from [WPILib Releases](https://github.com/wpilibsuite/allwpilib/releases)
   - Follow the installation guide for your platform

3. **Open in VS Code:**
   - Open the project folder in WPILib VS Code
   - The project should automatically configure dependencies

### Team Number Configuration
Set your team number in `.wpilib/wpilib_preferences.json`:
```json
{
  "teamNumber": 4079
}
```

## Building & Deployment

### Building the Code
```cmd
# Build the project
gradlew build

# Run code formatting
gradlew spotlessApply

# Generate documentation
gradlew dokkaHtml
```

### Deploying to Robot
```cmd
# Deploy to RoboRIO (robot must be connected)
gradlew deploy

# Deploy with debug information
gradlew deploy -Pdebug=true
```

### Simulation
```cmd
# Run robot simulation
gradlew simulateJava

# Run with GUI
gradlew simulateJava -Pheadless=false
```

## Development Guidelines

### Code Style
- **Language:** Primarily Kotlin, Java for compatibility where needed
- **Formatting:** Enforced by Spotless (run `gradlew spotlessApply`)
- **Naming:** Use descriptive names, follow Kotlin conventions
- **Documentation:** Document public APIs and complex logic

### Git Workflow
1. Create feature branches from `main`
2. Make atomic commits with descriptive messages
3. Test thoroughly before submitting pull requests
4. Code review required before merging

### Testing
- Unit tests for utility functions
- Simulation testing for robot behaviors
- Hardware-in-the-loop testing when possible

## Autonomous Modes

### Available Autonomous Routines
- **ScoreL4Left** - Score at L4 position from left side
- **ScoreL4Right** - Score at L4 position from right side
- **Multi-piece routines** - Various multi-game piece sequences

### Path Planning
- Uses PathPlanner for trajectory generation
- Custom A* pathfinding implementation for dynamic obstacles
- Real-time path optimization based on field conditions

## Controls

### Driver Controls
- **Left Stick:** Translation (X/Y movement)
- **Right Stick:** Rotation
- **Triggers:** Speed modulation
- **Buttons:** Field-centric toggle, auto-alignment

### Operator Controls
- **Elevator Control:** Manual and preset positioning
- **Intake/Outtake:** Game piece manipulation
- **Automated Sequences:** One-button scoring routines

## Logging & Telemetry

### AdvantageKit Integration
- Comprehensive data logging for match analysis
- Replay capability for debugging
- Real-time telemetry visualization

### Logged Data
- All subsystem states and sensor readings
- Command execution traces
- Performance metrics and diagnostics
- Vision processing results

### Viewing Logs
Use AdvantageScope to analyze logged data:
1. Connect to robot or load log files
2. Configure custom layouts for different analysis needs
3. Export data for further analysis

## Testing

### Simulation Testing
```cmd
# Run basic simulation
gradlew simulateJava

# Run with specific robot configuration
gradlew simulateJava -Probot=competition
```

### Hardware Testing
- Subsystem calibration routines available in `Calibration.kt`
- Individual subsystem testing through SmartDashboard
- Full system integration testing

## Contributing

### Getting Started
1. Fork the repository
2. Create a feature branch
3. Make your changes following the development guidelines
4. Test thoroughly
5. Submit a pull request

### Code Review Process
- All changes require review by team leads
- Automated checks must pass (build, formatting, basic tests)
- Hardware testing required for subsystem changes

### Issue Reporting
- Use GitHub Issues for bug reports and feature requests
- Provide detailed reproduction steps for bugs
- Include relevant log files when applicable

## License

This project is licensed under the terms of the WPILib License. See the [WPILib-License.md](WPILib-License.md) file for details.

---

**Team 4079** | [Website](https://team4079.org) | [GitHub](https://github.com/team4079)
