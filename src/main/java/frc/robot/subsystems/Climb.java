package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.ArmPidConstants;


public class Climb extends SubsystemBase  {

    public static boolean climbMode = false;
    
    CANSparkMax l_arm = new CANSparkMax(CAN_ID.CLIMB_LEFT, MotorType.kBrushless);
    CANSparkMax r_arm = new CANSparkMax(CAN_ID.CLIMB_RIGHT, MotorType.kBrushless);

   

    RelativeEncoder l_encoder;
    RelativeEncoder r_encoder;

    SparkMaxPIDController l_pidController;
    SparkMaxPIDController r_pidController;
    int smartMotionProfile = 1;

    public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxVel, minVel, maxRPM, maxAcc, allowedErr;

    double pid_setPoint = 0;

    public Climb() {
        l_arm.restoreFactoryDefaults();
        r_arm.restoreFactoryDefaults();
        l_arm.setInverted(false);
        r_arm.setInverted(true);
        l_arm.setSmartCurrentLimit(40);
        r_arm.setSmartCurrentLimit(40);
        l_arm.setClosedLoopRampRate(1);
        r_arm.setClosedLoopRampRate(1);

        l_arm.setIdleMode(IdleMode.kBrake);
        r_arm.setIdleMode(IdleMode.kBrake);

        l_encoder = l_arm.getEncoder();
        r_encoder = r_arm.getEncoder();

        l_encoder.setPosition(0);
        r_encoder.setPosition(0);

        l_pidController = l_arm.getPIDController();
        r_pidController = r_arm.getPIDController();

        // PID coefficients these will need to be tuned
        kP = ArmPidConstants.kP;
        kI = ArmPidConstants.kI;
        kD = ArmPidConstants.kD;
        kIz = ArmPidConstants.kIz;
        kFF = ArmPidConstants.kFF;
        kMaxOutput = ArmPidConstants.kMaxOutput;
        kMinOutput = ArmPidConstants.kMinOutput;
        maxRPM = ArmPidConstants.maxRPM;
        maxVel = ArmPidConstants.maxVel;
        minVel = ArmPidConstants.minVel;
        maxAcc = ArmPidConstants.maxAcc;
        allowedErr = ArmPidConstants.allowedErr;
        
        l_pidController.setP(kP, smartMotionProfile);
        l_pidController.setI(kI, smartMotionProfile);
        l_pidController.setD(kD, smartMotionProfile);
        l_pidController.setIZone(kIz, smartMotionProfile);
        l_pidController.setFF(kFF, smartMotionProfile);
        l_pidController.setOutputRange(kMinOutput, kMaxOutput, smartMotionProfile);

        r_pidController.setP(kP, smartMotionProfile);
        r_pidController.setI(kI, smartMotionProfile);
        r_pidController.setD(kD, smartMotionProfile);
        r_pidController.setIZone(kIz, smartMotionProfile);
        r_pidController.setFF(kFF, smartMotionProfile);
        r_pidController.setOutputRange(kMinOutput, kMaxOutput, smartMotionProfile);


        l_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionProfile);
        l_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
        l_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        l_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionProfile);

        r_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionProfile);
        r_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
        r_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        r_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionProfile);
    }
    
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbAmp", l_arm.getOutputCurrent());
        SmartDashboard.putNumber("HookAmp", r_arm.getOutputCurrent());
        SmartDashboard.putNumber("Right Arm Pos", r_encoder.getPosition());
        SmartDashboard.putNumber("Right Arm Vel", r_encoder.getVelocity());
        SmartDashboard.putNumber("Right Arm Current", r_arm.getOutputCurrent());
        SmartDashboard.putNumber("Left Arm Pos", l_encoder.getPosition());
        SmartDashboard.putNumber("Left Arm Vel", l_encoder.getVelocity());
        SmartDashboard.putNumber("Left Arm Current", l_arm.getOutputCurrent());
        SmartDashboard.putNumber("Arm Set Point", pid_setPoint);
        SmartDashboard.putBoolean("Climb Mode", climbMode);
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
        r_arm.set(lift - trim);
        l_arm.set(lift + trim);
        // if (!climbMode || (lift > 0.2 || trim > 0.2)){
        //     lift = deadBand(lift);
        //     trim = deadBand(trim);
        //     l_pidController.setReference(lift + trim, ControlType.kDutyCycle, smartMotionProfile);
        //     // l_arm.set(lift + trim);
        //     r_pidController.setReference(lift - trim, ControlType.kDutyCycle, smartMotionProfile);
        //     // r_arm.set(lift - trim);
        // }
        // } if (!climbMode && (lift < 0.2 && trim < 0.2 )) {
        //     lift = deadBand(lift);
        //     trim = deadBand(trim);
        //     l_pidController.setReference(lift + trim, ControlType.kDutyCycle, smartMotionProfile);
        //     // l_arm.set(lift + trim);
        //     r_pidController.setReference(lift - trim, ControlType.kDutyCycle, smartMotionProfile);
        // }
    }


    public void startArmSmartMotion() {
        l_pidController.setReference(pid_setPoint, ControlType.kSmartMotion, smartMotionProfile);
        r_pidController.setReference(pid_setPoint, ControlType.kSmartMotion, smartMotionProfile);
    }
   

    public void extendArms() {
        
        pid_setPoint+=16;
        
    }

    public void retractArms() {
        
        pid_setPoint-=16;
        
    }

    public void setArmPidSetPoint(double setPoint){
        pid_setPoint=setPoint;
    }



    public double deadBand(double raw){
        if (Math.abs(raw) > 0.2){
            return raw;
        }
        else{
            return 0.0;
        }
    }

    public void setClimbModeTrue(){
        climbMode = true;
    }

    public void setClimbModeFalse(){
        climbMode = false;

    }
}
