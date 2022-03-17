package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.BallTower;

public class AutoSmartShot extends CommandBase {
    private final Launcher m_Launcher;
    private final BallTower m_BallTower;
    private boolean hasBallLaunched = false;

    public AutoSmartShot(Launcher launcher, BallTower ballTower) {
        m_Launcher = launcher;
        m_BallTower = ballTower;
        addRequirements(m_Launcher, m_BallTower);
    }

    @Override
    public void initialize() {
        m_BallTower.stopTower();
        m_Launcher.autoMidTarmacShoot();
    }

    @Override
    public void execute() {
        if (m_BallTower.isBallDetected()) {
            hasBallLaunched = false;
        } else {
            m_BallTower.stopTower();
            m_Launcher.StopShoot();
            hasBallLaunched = true;
        }
        if (m_Launcher.isAtVelocity()) {
            m_BallTower.feedBallToLauncher();
        }
    }

    @Override
    public boolean isFinished() {
        if (hasBallLaunched) {
            return true;
        } else {
            return false;
        }
    }

}
