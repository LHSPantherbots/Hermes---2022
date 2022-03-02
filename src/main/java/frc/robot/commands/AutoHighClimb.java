package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;


import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.ClimbPivot;

public class AutoHighClimb extends SequentialCommandGroup {
 public AutoHighClimb(DriveSubsystem driveTrain, ClimbPivot climbPivot, Climb climb) {
    addCommands(
        new InstantCommand(() -> climb.setArmPidSetPoint(10), climb),
        new RunCommand(() -> climb.startArmSmartMotion(), climb)
            .withTimeout(.5),
        // ClimbPivot
        new RunCommand(climbPivot::armBack, climbPivot)
            .withTimeout(1.5),
        // 116
        new InstantCommand(() -> climb.setArmPidSetPoint(116), climb),
        new RunCommand(() -> climb.startArmSmartMotion(), climb)
            .withTimeout(2.5),
        // ClimbPivot
        new RunCommand(climbPivot::armForward, climbPivot)
        .withTimeout(1.5),
        // Check if contact??
        // 0-1
        new InstantCommand(() -> climb.setArmPidSetPoint(1), climb),
        new RunCommand(() -> climb.startArmSmartMotion(), climb)
        .withTimeout(2.5)
    );
    // new RunCommand(() -> Timer.delay(.5)),
    // 10
    // new InstantCommand(() -> climb.setArmPidSetPoint(10), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    //     .withTimeout(.5),
    // // ClimbPivot
    // new RunCommand(climbPivot::armBack, climbPivot)
    //     .withTimeout(1.5),
    // // 116
    // new InstantCommand(() -> climb.setArmPidSetPoint(116), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    //     .withTimeout(2.5),
    // // ClimbPivot
    // new RunCommand(climbPivot::armForward, climbPivot)
    //   .withTimeout(1.5),
    // // Check if contact??
    // // 0-1
    // new InstantCommand(() -> climb.setArmPidSetPoint(1), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    // .withTimeout(2.5),
    // // new RunCommand(() -> Timer.delay(.5)),
    // // 10
    // new InstantCommand(() -> climb.setArmPidSetPoint(10), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    //     .withTimeout(.5),
    // // ClimbPivot
    // new RunCommand(climbPivot::armBack, climbPivot)
    //   .withTimeout(1.5),
    // // 110
    // new InstantCommand(() -> climb.setArmPidSetPoint(110), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    // .withTimeout(2.6),
    // // ClimbPivot
    // new RunCommand(climbPivot::armForward, climbPivot)
    //   .withTimeout(1.5),
    // // 0
    // new InstantCommand(() -> climb.setArmPidSetPoint(0), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    //   .withTimeout(2.5),
    // // new RunCommand(() -> Timer.delay(.5)),
    // // 1 (is this needed?)
    // new InstantCommand(() -> climb.setArmPidSetPoint(1), climb),
    // new RunCommand(() -> climb.startArmSmartMotion(), climb)
    //     .withTimeout(.4)
 }
}
