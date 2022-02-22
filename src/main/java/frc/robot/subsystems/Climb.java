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
import frc.robot.Constants.SparkMaxPidConstants;


public class Climb extends SubsystemBase  {
    
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
        l_arm.setClosedLoopRampRate(2);
        r_arm.setClosedLoopRampRate(2);

        l_arm.setIdleMode(IdleMode.kBrake);
        r_arm.setIdleMode(IdleMode.kBrake);

        l_encoder = l_arm.getEncoder();
        r_encoder = r_arm.getEncoder();

        l_encoder.setPosition(0);
        r_encoder.setPosition(0);

        l_pidController = l_arm.getPIDController();
        r_pidController = r_arm.getPIDController();

        // PID coefficients these will need to be tuned
        kP = SparkMaxPidConstants.kPP; 
        kI =  SparkMaxPidConstants.kI;
        kD = SparkMaxPidConstants.kD;
        kIz = SparkMaxPidConstants.kIz;
        kFF = SparkMaxPidConstants.kFF;
        kMaxOutput = SparkMaxPidConstants.kMaxOutput;
        kMinOutput = SparkMaxPidConstants.kMinOutput;
        maxRPM = SparkMaxPidConstants.maxRPM;
        maxVel = SparkMaxPidConstants.maxVel;
        minVel = SparkMaxPidConstants.minVel;
        maxAcc = SparkMaxPidConstants.maxAcc;
        allowedErr = SparkMaxPidConstants.allowedErr;
        // kP = 5e-5; 
        // kI = 1e-6;
        // kD = 0; 
        // kIz = 0; 
        // kFF = 0.000156; 
        // kMaxOutput = 1; 
        // kMinOutput = -1;
        // maxRPM = 5700;

        // Smart Motion Coefficients
        // maxVel = 2000; // rpm
        // maxAcc = 1500;
        
        l_pidController.setP(kP);
        l_pidController.setI(kI);
        l_pidController.setD(kD);
        l_pidController.setIZone(kIz);
        l_pidController.setFF(kFF);
        l_pidController.setOutputRange(kMinOutput, kMaxOutput);

        r_pidController.setP(kP);
        r_pidController.setI(kI);
        r_pidController.setD(kD);
        r_pidController.setIZone(kIz);
        r_pidController.setFF(kFF);
        r_pidController.setOutputRange(kMinOutput, kMaxOutput);


        l_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionProfile);
        l_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
        l_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        l_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionProfile);

        r_pidController.setSmartMotionMaxVelocity(maxVel, smartMotionProfile);
        r_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
        r_pidController.setSmartMotionMaxAccel(maxAcc, smartMotionProfile);
        r_pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionProfile);

        SmartDashboard.putNumber("Arm P Gain", kP);
        SmartDashboard.putNumber("Arm I Gain", kI);
        SmartDashboard.putNumber("Arm D Gain", kD);
        SmartDashboard.putNumber("Arm I Zone", kIz);
        SmartDashboard.putNumber("Arm Feed Forward", kFF);
        SmartDashboard.putNumber("Arm Max Output", kMaxOutput);
        SmartDashboard.putNumber("Arm Min Output", kMinOutput);

        // display Smart Motion coefficients
        SmartDashboard.putNumber("Arm Max Velocity", maxVel);
        SmartDashboard.putNumber("Arm Min Velocity", minVel);
        SmartDashboard.putNumber("Arm Max Acceleration", maxAcc);
        SmartDashboard.putNumber("Arm Allowed Closed Loop Error", allowedErr);

        SmartDashboard.putNumber("Right Arm Pos", r_encoder.getPosition());
        SmartDashboard.putNumber("Left Arm Pos", l_encoder.getPosition());
    }
    
    
    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimbAmp", l_arm.getOutputCurrent());
        SmartDashboard.putNumber("HookAmp", r_arm.getOutputCurrent());
        SmartDashboard.putNumber("Right Arm Pos", r_encoder.getPosition());
        SmartDashboard.putNumber("Left Arm Pos", l_encoder.getPosition());
        SmartDashboard.putNumber("Arm Set Point", pid_setPoint);
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


    public void startArmSmartMotion() {
        double p = SmartDashboard.getNumber("Arm P Gain", 0);
        double i = SmartDashboard.getNumber("Arm I Gain", 0);
        double d = SmartDashboard.getNumber("Arm D Gain", 0);
        double iz = SmartDashboard.getNumber("Arm I Zone", 0);
        double ff = SmartDashboard.getNumber("Arm Feed Forward", 0);
        double max = SmartDashboard.getNumber("Arm Max Output", 0);
        double min = SmartDashboard.getNumber("Arm Min Output", 0);
        double maxV = SmartDashboard.getNumber("Arm Max Velocity", 0);
        double minV = SmartDashboard.getNumber("Arm Min Velocity", 0);
        double maxA = SmartDashboard.getNumber("Arm Max Acceleration", 0);
        double allE = SmartDashboard.getNumber("Arm Allowed Closed Loop Error", 0);

        // if PID coefficients on SmartDashboard have changed, write new values to controller
        if((p != kP)) { l_pidController.setP(p); r_pidController.setP(p); kP = p; }
        if((i != kI)) { l_pidController.setI(i); r_pidController.setI(i); kI = i; }
        if((d != kD)) { l_pidController.setD(d); r_pidController.setD(d); kD = d; }
        if((iz != kIz)) { l_pidController.setIZone(iz); r_pidController.setIZone(iz); kIz = iz; }
        if((ff != kFF)) { l_pidController.setFF(ff); r_pidController.setFF(ff); kFF = ff; }
        if((max != kMaxOutput) || (min != kMinOutput)) { 
            l_pidController.setOutputRange(min, max); 
            r_pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max;
        }
        if((maxV != maxVel)) { l_pidController.setSmartMotionMaxVelocity(maxV,0); r_pidController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
        if((minV != minVel)) { l_pidController.setSmartMotionMinOutputVelocity(minV,0); r_pidController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
        if((maxA != maxAcc)) { l_pidController.setSmartMotionMaxAccel(maxA,0); r_pidController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
        if((allE != allowedErr)) { l_pidController.setSmartMotionAllowedClosedLoopError(allE,0); r_pidController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }
        
        l_pidController.setReference(pid_setPoint, ControlType.kSmartMotion);
        r_pidController.setReference(pid_setPoint, ControlType.kSmartMotion);
    }
   

    public void extendArms() {
        
        pid_setPoint+=16;
        
    }

    public void retractArms() {
        
        pid_setPoint-=16;
        
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




