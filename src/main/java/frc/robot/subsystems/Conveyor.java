package frc.robot.subsystems;

import frc.robot.RobotContainer;
import frc.robot.commands.*;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;


public class Conveyor extends SubsystemBase{
    
    public double timerDelaySetpoint = 0.5;
    public double timerDelayValue = 0.0;
    public boolean wasBallDetected = false; //was there a ball detected in the previous loop?

    // DigitalInput DownS = new DigitalInput(2);
    // DigitalInput UpS = new DigitalInput(0);

    CANSparkMax Conveyer = new CANSparkMax(8, MotorType.kBrushless);

  
 
   

  public Conveyor() {
      
      Conveyer.setInverted(true);
  }




    @Override
    public void periodic() {
    // SmartDashboard.putBoolean("Intake Sensor", IsBallD());
    // SmartDashboard.putBoolean("Launcher Sensor", IsBallU());
    //SmartDashboard.putNumber("Timer Delay", timerDelayValue);

    }

    public void run() {
        Conveyer.set(.2);
        
    }

    public void stop() {
        Conveyer.stopMotor();
        
    }


    

    public void conveyerForward() {
        timerDelayValue = timerDelayValue - 0.01;

        if(!wasBallDetected){ //if there is not ball detected reset the delay timer
            timerDelayValue = timerDelaySetpoint;
        }
        
        if(RobotContainer.ballEjector.isBallDetected()){
            wasBallDetected = true;
        }
        else{
            wasBallDetected = false;
        }
        

        if (Climb.climbMode || (RobotContainer.ballEjector.hasTwoBalls() && (timerDelaySetpoint<0) )){ // stops conveyer if climbMode is enabled
            Conveyer.stopMotor();
        } else {
            Conveyer.set(.2);
        }
        
    }

    public void conveyerBackward() {
        Conveyer.set(-.2);
    }

    public void conveyerStop() {
        Conveyer.stopMotor();
    }

    

    

}




