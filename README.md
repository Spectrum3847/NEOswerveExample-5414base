# 2023CompetitionBot
 FRC Team 5414's code for the 2023 competition robot, Rooty ~~(or Roosevelt)~~. The code is written in Java and uses WPILib's Java command-based structure.
 
## Code Highlights
- Field-Centric Swerve Drive
  
  The robot's drivetrain is a [standard swerve drivetrain with field-centric control](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/Drivetrain.java) using double NEO SDS Mk4i modules. The drivetrain uses encoders, a Pigeon 2 gyro, and odometry to control movement during the autonomous and teleoperated phases. The rotation of the drivetrain can be controlled either through [speed](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/Drivetrain.java#L111) or [heading](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/Drivetrain.java#L140).

- Cube Shooter, One DOF Arm, "Big Stick", LED Indicators

  The robot uses WPILib subsystems and enums to effectively create a state machine that controls each mechanism. The robot features a [cube shooter](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/Shooter.java) that uses fixed speeds using velocity PID control and a beam break sensor to detect cubes. The [arm](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/Arm.java) uses positional PID control for each setpoint, as does the "[big stick](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/BigStick.java)," which extends the range for shooting behind the charge station. [LEDs](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/subsystems/LEDStrip.java) also indicate the presence of a cube, the shooter mode, and indicate to the human player the desired game piece.

- Autonomous Path Following

  The robot uses [Team 3015's PathPlanner](https://github.com/mjansen4857/pathplanner), a motion profile generator for FRC robots, to generate and follow autonomous trajectories. [Autonomous routines](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/commands/Autos.java) are created using sequential command groups and selected through sendable choosers in SmartDashboard.

- Launchpad

  The operator uses a Novation MK2/MK3 launchpad to control much of the robot's mechanisms. It operates from the command prompt by running a Python project that uses MIDI, pygame, and pynetworktables which connects the launchpad to the computer and creates a NetworkTable. A [Launchpad Button class](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/lib/drivers/LaunchpadButton.java) that extends the Trigger class allows the implementation of commands like that of Joystick buttons onto the [Launchpad](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/lib/drivers/Launchpad.java).

- Fully Automatic Balancing

  The drivetrain uses PID controllers and data from the gyro's pitch and roll to [automatically balance](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/commands/AutoBalance.java) from any orientation during the autonomous phase of the match.

## Variable Naming Conventions
- XXX_XXX (i.e. `WHEEL_DIAMETER`): All [constants](https://github.com/Pearadox/2023CompetitionBot/blob/main/src/main/java/frc/robot/Constants.java) (screaming snake case)
- kXxx (i.e. `kZero`): All enum constants for states
- xxXxxXxx (i.e. `autoStartingSideChooser`): All private instance variables (camel case)
