package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends CommandBase {
    public Command lowerIntake;
    public Command RunIntake;
    public Command RaiseIntake;

    public IntakeCommand(Intake intakeSubsys) {

    }

    // public Command lowerIntake() {
    //     intakeSubsys.
    // }

    // public SequentialCommandGroup DropAndIntake() {
    //     // new SequentialCommandGroup(commands)
    // }
}
