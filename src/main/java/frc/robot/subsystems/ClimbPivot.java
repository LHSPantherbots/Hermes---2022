package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PH_Channel;
import frc.robot.Constants.CAN_ID;


public class ClimbPivot extends SubsystemBase  {
  

    DoubleSolenoid armSolenoid = new DoubleSolenoid(26,PneumaticsModuleType.REVPH, PH_Channel.CLIMB_A, PH_Channel.CLIMB_B);



    public ClimbPivot() {
       

       
    }
    
    
    @Override
    public void periodic() {
        
    }



    public void armForward() {
        armSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void armBack() {
        armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

  



  
}




