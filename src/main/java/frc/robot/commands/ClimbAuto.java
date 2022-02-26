package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ClimbPivot;

public class ClimbAuto extends SequentialCommandGroup {
 public ClimbAuto(DriveSubsystem driveSubsystem, ClimbPivot climbPivot, Climb climb) {
    addCommands(
        new InstantCommand(() -> System.out.println("ClimbAuto Command Triggered!")),
        new InstantCommand(() -> Climb.setClimbMode()),
        // add commands (numbers are arm possitions)
        // 95
        new RunCommand(() -> climb.setArmPidSetPoint(95), climb)
            .withTimeout(2),
        // drive back short amount
        new RunCommand(() -> driveSubsystem.teleopDrive(.1, 0), driveSubsystem)
            .withTimeout(.5),
        // 89
        new RunCommand(() -> climb.setArmPidSetPoint(89), climb)
            .withTimeout(1),
        // Check if contact??
        // 0-1
        new RunCommand(() -> climb.setArmPidSetPoint(1), climb)
            .withTimeout(2),
        // 10
        new RunCommand(() -> climb.setArmPidSetPoint(10), climb)
            .withTimeout(.5),
        // ClimbPivot
        new RunCommand(climbPivot::armBack, climbPivot),
        // 116
        new RunCommand(() -> climb.setArmPidSetPoint(116), climb)
            .withTimeout(2),
        // ClimbPivot
        new RunCommand(climbPivot::armForward, climbPivot),
        // Check if contact??
        // 0-1
        new RunCommand(() -> climb.setArmPidSetPoint(1), climb)
            .withTimeout(2),
        // 10
        new RunCommand(() -> climb.setArmPidSetPoint(10), climb)
            .withTimeout(.5),
        // ClimbPivot
        new RunCommand(climbPivot::armBack, climbPivot),
        // 110
        new RunCommand(() -> climb.setArmPidSetPoint(110), climb)
        .withTimeout(2),
        // ClimbPivot
        new RunCommand(climbPivot::armForward, climbPivot),
        // 0
        new RunCommand(() -> climb.setArmPidSetPoint(0), climb)
            .withTimeout(2),
        // 1 (is this needed?)
        new RunCommand(() -> climb.setArmPidSetPoint(1), climb)
            .withTimeout(.2)
        );
 }
}
