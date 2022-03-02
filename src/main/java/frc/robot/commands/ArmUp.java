package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ClimbPivot;

public class ArmUp extends SequentialCommandGroup {
 public ArmUp(DriveSubsystem driveTrain, ClimbPivot climbPivot, Climb climb) {
    addCommands(
        new InstantCommand(() -> System.out.println("ClimbAuto Command Triggered!")),
        new InstantCommand(() -> RobotContainer.climb.setClimbModeTrue(), RobotContainer.climb),
        
        // add commands (numbers are arm possitions)
        // 95
        new InstantCommand(() -> climb.setArmPidSetPoint(95), climb),
        new RunCommand(() -> climb.startArmSmartMotion(), climb)
            .withTimeout(2.5)
        
    );
 }
}
