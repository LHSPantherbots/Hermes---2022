package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.RIO_Channels_DIO;



public class BallTower extends SubsystemBase {
    CANSparkMax towerRoller = new CANSparkMax(7, MotorType.kBrushless);
    CANSparkMax towerBelts = new CANSparkMax(12, MotorType.kBrushless);
    DigitalInput beamBreak = new DigitalInput(RIO_Channels_DIO.TOWER_BEAM_BREAK);
    boolean launcherAtSpeed;
    // DigitalInput Colorsense = new DigitalInput(4); 

    public BallTower() {
        towerRoller.restoreFactoryDefaults();
        towerBelts.restoreFactoryDefaults();

        towerBelts.setOpenLoopRampRate(.7);
        towerRoller.setOpenLoopRampRate(.7);

        towerRoller.setSmartCurrentLimit(30);
        towerBelts.setSmartCurrentLimit(30);
        towerBelts.setInverted(true);
        towerRoller.setInverted(false);
    }

    public void liftBall() {
        towerRoller.set(.8);
        if (beamBreak.get()) {
            towerBelts.set(.2);
        } else {
            towerBelts.set(0);
        }
        
    }

    @Override
    public void periodic() {
        launcherAtSpeed = SmartDashboard.getBoolean("At Set Velocity", false);
        SmartDashboard.putBoolean("BeamBreak", isBallDetected());
    }

    public void feedBallToLauncher() {
        if (!beamBreak.get()) {
        // if (launcherAtSpeed && !beamBreak.get()) {
            towerBelts.set(.9);
        }
    }

    public void stopTower() {
        towerRoller.stopMotor();
        towerBelts.stopMotor();
    }

    public void stopBelts() {
        towerBelts.stopMotor();
    }

    public void lowerBall() {
        towerRoller.set(-.2);
        towerBelts.set(-.2);
    }

    public boolean isBallDetected(){
        return !beamBreak.get();
    }

    public void autoTower(){

        if (!isBallDetected() && !Climb.climbMode){
            liftBall();
        }
        else{
            stopTower();
        }
    }



}
