package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallTower;

public class AutoSmartTower extends CommandBase {
    private final BallTower m_BallTower;
    private boolean hasBallReachedTopBeamBreak = false;

    public AutoSmartTower(BallTower ballTower) {
        m_BallTower = ballTower;
        addRequirements(m_BallTower);
    }

    @Override
    public void initialize() {
        m_BallTower.stopTower();
    }

    @Override
    public void execute() {
        if (m_BallTower.isBallDetected()) {
            m_BallTower.stopTower();
            hasBallReachedTopBeamBreak = true;
        } else {
            m_BallTower.liftBall();
        }
    }

    @Override
    public boolean isFinished() {
        if (hasBallReachedTopBeamBreak) {
            return true;
        } else {
            return false;
        }
    }

    
}
