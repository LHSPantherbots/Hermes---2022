# FRC Team 2582 Pantherbots - 2022 RapidReact

## CAN ID Table
| Subsystem | Device Type       | Device            | CAN ID | Notes            | 
|-----------|-------------------|-------------------|--------|------------------|
|Drive Train|CANSparkMax        |LeftLeader         |3       |                  |
|Drive Train|CANSparkMax        |LeftFollow         |15       |                  |
|Drive Train|CANSparkMax        |RightLeader        |5      |                  |
|Drive Train|CANSparkMax        |RightFollower      |6       |                  |
|Imu        |Pigeon             |pidgey             |4      |Child of talonIntake, Created in RobotContainer |
|Intake     |TalonSRX           |talonIntake        |4      |Created in RobotContainer |
|Intake     |CANSparkMax        |Conveyer           |8      |               |
|Launcher   |CANSparkMax        |launcherLeader     | 9      |                   |
|Launcher   |CANSparkMax        |launcherFollower   | 10     |                   |
|Climb Hook |CANSparkMax        |climb              | 13      | May need to change ID |
|Climb Hook |CANSparkMax        |hook               | 14      | Will need to change ID, conflicts with PDP |
|Ball Ejector |CANSparkMax      |BallEject          | 11     |                   |
|           | PDP               |                   | 2      |                   |
|BallTower |CANSparkMax         |towerRoller        | 7     |                  |
|BallTower |CANSparkMax         |towerBelts         | 12     |                  |
|CompressorSubsystem|Compressor |compressor         | 26    | All pneumatics objects must use this can ID, default REVPH id of 2 conflicted with PDP |


## Solenoid Channel Table
| Subsystem | Single/Double     | Channel(s)        | Device            | Notes         |
|-----------|-------------------|-------------------|-------------------|---------------|
| Intake    | Double            | 1,2                  | intakeSolenoid    |  |
| Launcher  | Single            | 3                  |                   | Not defined yet, but needed for adjustable hood |
| Climb Hook    | Double        | 4,5                  |                   | |

## DIO Channel Table
| Subsystem | Device            | Channel | Notes            | 
|-----------|-------------------|---------|------------------|
|BallTower  | beamBreak         | 1       |                  |

## Controls/Button Mapping
- Controller0
    - Button A: Runs launcher flywheel at slow speed (40% power)
    - Button B: Runs launcher flywheel at high speed (70% power)
    - Button Y: Ejects balls
        - Runs Conveyer Forward
        - Runs Ejector Wheel Backwards
    - Button RB: Lifts Ball in Tower
        - Runs Conveyer Forward
        - Runs Ejector Wheel Forward
        - Runs Tower Wheel and Belts Forward(up)
    - Button LB: Lowers Ball in Tower (*maybe be removed in the future)
        - Runs Conveyer Backward
        - Runs Ejector Wheel Backward
        - Runs Tower Wheel and Belts Backward(down)
    - D-Pad:
        - Right: Toggle PID Closed Loop Flywheel speed control On/Off (default RPM 3500)
        - Up: Increases Closed Loop RPM target by 10 rpm
        - Down: Decreases Closed Loop RPM target y 10 rpm
    - Axis 0 (Left stick x axis): Drivetrain Turn Left/Right
    - Axis 1 (Left stick y axis): Drivetrain Forward/Reverse
