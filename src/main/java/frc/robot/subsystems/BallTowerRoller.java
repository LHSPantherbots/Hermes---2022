package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_ID;
import frc.robot.RobotContainer;
import frc.robot.Constants.RIO_Channels_DIO;


public class BallTowerRoller extends SubsystemBase {
    CANSparkMax towerRoller = new CANSparkMax(CAN_ID.BALL_TOWER_ROLLER, MotorType.kBrushless);
    public double towerRollerDelaySetpoint = 0.1;
    public double towerRollerDelayValue = 0.0;
    public boolean isworkautobelts = false;

    public BallTowerRoller(){
        towerRoller.restoreFactoryDefaults();
        towerRoller.setOpenLoopRampRate(.7);

        towerRoller.setSmartCurrentLimit(20);
        towerRoller.setInverted(false);
    }

    public void runTowerRoller(){
        towerRoller.set(.8);
    }



    @Override
    public void periodic() {
        
    }
}



