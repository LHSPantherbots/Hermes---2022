package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class AutoCommand  extends CommandBase {
    public AutoCommand(DriveSubsystem driveSubsystem) {
        driveSubsystem.teleopDrive(0.0, 0.0);
    }
}
