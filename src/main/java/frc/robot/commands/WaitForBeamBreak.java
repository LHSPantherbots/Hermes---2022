package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallTower;

public class WaitForBeamBreak extends CommandBase {
    private final BallTower m_BallTower;
    public WaitForBeamBreak(BallTower subsystem) {
        m_BallTower = subsystem;
        addRequirements(m_BallTower);
    }

    @Override
    public boolean isFinished() {
        if (m_BallTower.isBallDetected()){
            return true;
        } else {
            return false;
        }
    }
}
