package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.SparkMaxPidConstants;


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
        // kP = SparkMaxPidConstants.kPP;
        kP = 0.0000035; 
        // kI =  SparkMaxPidConstants.kI;
        kI = 0.0;
        // kD = SparkMaxPidConstants.kD;
        kD = 0.00002;
        kIz = SparkMaxPidConstants.kIz;
        // kFF = SparkMaxPidConstants.kFF;
        kFF = 0.000165;
        kMaxOutput = SparkMaxPidConstants.kMaxOutput;
        // kMinOutput = SparkMaxPidConstants.kMinOutput;
        kMinOutput = SparkMaxPidConstants.kMaxOutput*-1;
        maxRPM = SparkMaxPidConstants.maxRPM;
        maxVel = SparkMaxPidConstants.maxVel;
        // minVel = SparkMaxPidConstants.minVel;
        minVel = SparkMaxPidConstants.maxVel*-1;
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
        // l_pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionProfile);
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
        SmartDashboard.putNumber("Right Arm Vel", r_encoder.getVelocity());
        SmartDashboard.putNumber("Right Arm Current", r_arm.getOutputCurrent());
        SmartDashboard.putNumber("Left Arm Pos", l_encoder.getPosition());
        SmartDashboard.putNumber("Left Arm Vel", l_encoder.getVelocity());
        SmartDashboard.putNumber("Left Arm Current", l_arm.getOutputCurrent());
        SmartDashboard.putNumber("Arm Set Point", pid_setPoint);

        SmartDashboard.putNumber("Arm P Gain", l_pidController.getP(smartMotionProfile));
        SmartDashboard.putNumber("Arm I Gain", l_pidController.getI(smartMotionProfile));
        SmartDashboard.putNumber("Arm D Gain", l_pidController.getD(smartMotionProfile));
        SmartDashboard.putNumber("Arm I Zone", l_pidController.getIZone(smartMotionProfile));
        SmartDashboard.putNumber("Arm Feed Forward", l_pidController.getFF(smartMotionProfile));
        SmartDashboard.putNumber("Arm Max Output", l_pidController.getSmartMotionMaxVelocity(smartMotionProfile));
        SmartDashboard.putBoolean("Climb Mode", climbMode);
        // SmartDashboard.putNumber("Arm Min Output", l_pidController.getSmartMotionMinOutputVelocity(smartMotionProfile));
        // SmartDashboard.putNumber("Arm PID SetPoint", l_pidController.)
    }


    public void manualleftClimb(double move){
        l_arm.set(move);
    }

    public void manualrightClimb(double move){
        r_arm.set(move);
    }

    public void manualClimb(double lift, double trim){
        if (!climbMode){
            lift = deadBand(lift);
            trim = deadBand(trim);
            l_arm.set(lift + trim);
            r_arm.set(lift - trim);
        }
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
        if((p != kP)) { l_pidController.setP(p, smartMotionProfile); r_pidController.setP(p, smartMotionProfile); kP = p; }
        if((i != kI)) { l_pidController.setI(i, smartMotionProfile); r_pidController.setI(i, smartMotionProfile); kI = i; }
        if((d != kD)) { l_pidController.setD(d, smartMotionProfile); r_pidController.setD(d, smartMotionProfile); kD = d; }
        if((iz != kIz)) { l_pidController.setIZone(iz, smartMotionProfile); r_pidController.setIZone(iz, smartMotionProfile); kIz = iz; }
        if((ff != kFF)) { l_pidController.setFF(ff, smartMotionProfile); r_pidController.setFF(ff, smartMotionProfile); kFF = ff; }
        // if((max != kMaxOutput) || (min != kMinOutput)) { 
        //     l_pidController.setOutputRange(min, max, smartMotionProfile); 
        //     r_pidController.setOutputRange(min, max, smartMotionProfile); 
        //     kMinOutput = min; kMaxOutput = max;
        // }
        if((maxV != maxVel)) { l_pidController.setSmartMotionMaxVelocity(maxV,smartMotionProfile); r_pidController.setSmartMotionMaxVelocity(maxV,smartMotionProfile); maxVel = maxV; }
        if((minV != minVel)) { l_pidController.setSmartMotionMinOutputVelocity(minV,smartMotionProfile); r_pidController.setSmartMotionMinOutputVelocity(minV,smartMotionProfile); minVel = minV; }
        if((maxA != maxAcc)) { l_pidController.setSmartMotionMaxAccel(maxA,smartMotionProfile); r_pidController.setSmartMotionMaxAccel(maxA,smartMotionProfile); maxAcc = maxA; }
        if((allE != allowedErr)) { l_pidController.setSmartMotionAllowedClosedLoopError(allE,smartMotionProfile); r_pidController.setSmartMotionAllowedClosedLoopError(allE,smartMotionProfile); allowedErr = allE; }
        
        // l_pidController.setReference(pid_setPoint, ControlType.kSmartMotion);
        l_pidController.setReference(pid_setPoint, ControlType.kSmartMotion, smartMotionProfile);
        r_pidController.setReference(pid_setPoint, ControlType.kSmartMotion, smartMotionProfile);
        // pid_setPoint=0;
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
        if (Math.abs(raw) > 0.1){
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
