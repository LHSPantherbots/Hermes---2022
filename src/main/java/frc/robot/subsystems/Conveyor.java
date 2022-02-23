package frc.robot.subsystems;

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

    }

    public void run() {
        Conveyer.set(.2);
        
    }

    public void stop() {
        Conveyer.stopMotor();
        
    }


    public void conveyerForward() {
        if (!RobotContainer.climbMode){ // stops conveyer if climbMode is enabled
            Conveyer.set(.2);
        } else {
            Conveyer.stopMotor();
        }
        
    }

    public void conveyerBackward() {
        Conveyer.set(-.2);
    }

    public void conveyerStop() {
        Conveyer.stopMotor();
    }

    

}




