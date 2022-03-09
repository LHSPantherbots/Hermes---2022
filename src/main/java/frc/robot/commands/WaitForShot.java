package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.BallTower;

public class WaitForShot extends CommandBase {
    private final Launcher m_Launcher;
    private final BallTower m_BallTower;
    public WaitForShot(Launcher launcher, BallTower ballTower) {
        m_Launcher = launcher;
        m_BallTower = ballTower;
        addRequirements(m_Launcher, m_BallTower);
    }

    @Override
    public boolean isFinished() {
        if (!m_Launcher.isAtVelocity()) {
            m_BallTower.stopBelts();
            return true;
        } else {
            return false;
        }
    }
}
