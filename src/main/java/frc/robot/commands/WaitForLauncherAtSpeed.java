package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;

public class WaitForLauncherAtSpeed extends CommandBase {
    private final Launcher m_Launcher;
    public WaitForLauncherAtSpeed(Launcher subsystem) {
        m_Launcher = subsystem;
        addRequirements(m_Launcher);
    }

    @Override
    public boolean isFinished() {
        if (m_Launcher.isAtVelocity()){
            return true;
        } else {
            return false;
        }
    }
}
