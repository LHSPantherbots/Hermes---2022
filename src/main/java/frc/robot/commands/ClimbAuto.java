package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.robot.RobotContainer;

public class ClimbAuto extends SequentialCommandGroup {
 public ClimbAuto() {
    addCommands(
        new InstantCommand(() -> RobotContainer.setClimbMode()),
        // add commands (numbers are arm possitions)
        // 95
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(95), RobotContainer.climb)
            .withTimeout(2),
        // drive back short amount
        new RunCommand(() -> RobotContainer.driveTrain.teleopDrive(.1, 0), RobotContainer.driveTrain)
            .withTimeout(.5),
        // 89
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(89), RobotContainer.climb)
            .withTimeout(1),
        // Check if contact??
        // 0-1
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(1), RobotContainer.climb)
            .withTimeout(2),
        // 10
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(10), RobotContainer.climb)
            .withTimeout(.5),
        // ClimbPivot
        new RunCommand(RobotContainer.climbPivot::armBack, RobotContainer.climbPivot),
        // 116
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(116), RobotContainer.climb)
            .withTimeout(2),
        // ClimbPivot
        new RunCommand(RobotContainer.climbPivot::armForward, RobotContainer.climbPivot),
        // Check if contact??
        // 0-1
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(1), RobotContainer.climb)
            .withTimeout(2),
        // 10
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(10), RobotContainer.climb)
            .withTimeout(.5),
        // ClimbPivot
        new RunCommand(RobotContainer.climbPivot::armBack, RobotContainer.climbPivot),
        // 110
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(110), RobotContainer.climb)
        .withTimeout(2),
        // ClimbPivot
        new RunCommand(RobotContainer.climbPivot::armForward, RobotContainer.climbPivot),
        // 0
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(0), RobotContainer.climb)
            .withTimeout(2),
        // 1 (is this needed?)
        new RunCommand(() -> RobotContainer.climb.setArmPidSetPoint(1), RobotContainer.climb)
            .withTimeout(.2)
        );
 }   
}
