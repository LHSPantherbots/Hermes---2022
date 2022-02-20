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


public class Climb extends SubsystemBase  {
    
    CANSparkMax l_arm = new CANSparkMax(CAN_ID.CLIMB_LEFT, MotorType.kBrushless);
    CANSparkMax r_arm = new CANSparkMax(CAN_ID.CLIMB_RIGHT, MotorType.kBrushless);

   

    RelativeEncoder l_Encoder;
    RelativeEncoder r_Encoder;

    SparkMaxPIDController l_pidController;
    SparkMaxPIDController r_pidController;


    public Climb() {
        l_arm.restoreFactoryDefaults();
        r_arm.restoreFactoryDefaults();
        l_arm.setInverted(false);
        r_arm.setInverted(true);
        l_arm.setSmartCurrentLimit(40);
        r_arm.setSmartCurrentLimit(40);
        l_arm.setClosedLoopRampRate(2);
        r_arm.setClosedLoopRampRate(2);

        l_arm.setIdleMode(IdleMode.kBrake);
        r_arm.setIdleMode(IdleMode.kBrake);

        l_Encoder = l_arm.getEncoder();
        r_Encoder = r_arm.getEncoder();

        l_pidController = l_arm.getPIDController();
        r_pidController = r_arm.getPIDController();
    }
    
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbAmp", l_arm.getOutputCurrent());
        SmartDashboard.putNumber("HookAmp", r_arm.getOutputCurrent());
    }


    public void manualleftClimb(double move){
        l_arm.set(move);
    }

    public void manualrightClimb(double move){
        r_arm.set(move);
    }

    public void manualClimb(double lift, double trim){
        lift = deadBand(lift);
        trim = deadBand(trim);
        l_arm.set(lift + trim);
        r_arm.set(lift - trim);
    }



   

    public void extendArms() {
        l_pidController.setReference(16, ControlType.kPosition);
        r_pidController.setReference(16, ControlType.kPosition);
    }

    public void retractArms() {
        l_pidController.setReference(-16, ControlType.kPosition);
        r_pidController.setReference(-16, ControlType.kPosition);
    }



    public double deadBand(double raw){
        if (Math.abs(raw) > 0.1){
            return raw;
        }
        else{
            return 0.0;
        }
    }
}




