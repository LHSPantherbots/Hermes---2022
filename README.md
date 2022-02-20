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
|Climb Hook |CANSparkMax        |climb left         | 13      |   |
|Climb Hook |CANSparkMax        |climb right        | 14      |  |
|Ball Ejector |CANSparkMax      |BallEject          | 11     |                   |
|           | PDP               |                   | 2      |                   |
|BallTower |CANSparkMax         |towerRoller        | 7     |                  |
|BallTower |CANSparkMax         |towerBelts         | 12     |                  |
|CompressorSubsystem|Compressor |compressor         | 26    | All pneumatics objects must use this can ID, default REVPH id of 2 conflicted with PDP |


## Solenoid Channel Table
| Subsystem | Single/Double     | Channel(s)        | Device            | Notes         |
|-----------|-------------------|-------------------|-------------------|---------------|
| Intake    | Double            | 3,4                  | intakeSolenoid    |  |
| Launcher  | Single            | 0                |                   |  |
| Climb Pivot    | Double        | 1,2                 |                   | |

## DIO Channel Table
| Subsystem | Device            | Channel | Notes            | 
|-----------|-------------------|---------|------------------|
|BallTower  | beamBreak         | 0       |                  |
|BallEjector| redInput          | 8       |                  |
|BallEjector| blueInput         | 9       |                  |

## Controls/Button Mapping
- Controller0
    - Button A: Decrease launcher RPM setpoint
    - Button B: Closes launcher hood
    - Button X: Increase launcher RPM setpoint
    - Button Y: Opens lanucher hood
    - Button RB: Lifts Ball in Tower
        - Runs Conveyer Forward
        - Runs Ejector Wheel Forward
        - Runs Tower Wheel and Belts Forward(up)
    - Button LB: Launches Ball
        - Runs Tower Belts Upward if there is a ball in the beam break
    - D-Pad:
        - Right: Fender low shot
        - Down: Fender high shot
        - Left: Mid tarmac shot
        - Up: Long tarmac shot
    - Left stick Y axis: Drivetrain Forward/Reverse
    - Right stick X axis: Drivetrain Turn Left/Right

- Controller1
    - Button A: Intake Rollers Backward
         - When held  
    - Button B: Closes launcher hood
    - Button X: Intake Rollers Forward
         - When held
    - Button Y: Opens lanucher hood
    - Button RB: Climber Pivot
        - When held
    - Button LB: Intake Down
        - when held
    - Left stick Y axis: Climb Arms Up/Down
    - Right stick X axis: Climb Arms Adjust
