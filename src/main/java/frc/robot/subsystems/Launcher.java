package frc.robot.subsystems;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.PH_Channel;


public class Launcher extends SubsystemBase{
    
    CANSparkMax launcherLeader = new CANSparkMax(9, MotorType.kBrushless);
    CANSparkMax launcherFollower = new CANSparkMax(10, MotorType.kBrushless);

    RelativeEncoder launcherEncoder;
    Solenoid hoodSolenoid = new Solenoid(26, PneumaticsModuleType.REVPH, PH_Channel.HOOD);
   
    
   

    private SparkMaxPIDController pidController;
   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;

   private double velocitySetpoint = 0.0;
   private double lastSetpoint = 3500;
    

    public Launcher() {


//sets up follower and makes the direction inverted to the leader
launcherLeader.restoreFactoryDefaults();

launcherFollower.restoreFactoryDefaults();

launcherLeader.setSmartCurrentLimit(60);

launcherFollower.setSmartCurrentLimit(60);

launcherLeader.setClosedLoopRampRate(2);
launcherFollower.setClosedLoopRampRate(2);

launcherLeader.setIdleMode(IdleMode.kCoast);
launcherFollower.setIdleMode(IdleMode.kCoast);

 //sets up follower and makes the direction inverted to the leader
launcherFollower.follow(launcherLeader, true);

 //Sets up endcoders
 launcherEncoder = launcherLeader.getEncoder();

 pidController = launcherLeader.getPIDController();

 // PID coefficients these will need to be tuned
 kP = 0.00025; //5e-5; 
 kI =  0;//1e-6;
 kD = 0.0004; 
 kIz = 0; 
 kFF = 0.00018; 
 kMaxOutput = 1; 
 kMinOutput = -1;
 maxRPM = 5700;
 allowableError = 100; //Lets the system known when the velocity is close enough to launch

 // set PID coefficients
 pidController.setP(kP);
 pidController.setI(kI);
 pidController.setD(kD);
 pidController.setIZone(kIz);
 pidController.setFF(kFF);
 pidController.setOutputRange(kMinOutput, kMaxOutput);

SmartDashboard.putNumber("P Gain", kP);
SmartDashboard.putNumber("I Gain", kI);
SmartDashboard.putNumber("D Gain", kD);
SmartDashboard.putNumber("I Zone", kIz);
SmartDashboard.putNumber("Feed Forward", kFF);
SmartDashboard.putNumber("Max Output", kMaxOutput);
SmartDashboard.putNumber("Min Output", kMinOutput);


    }

    @Override
    public void periodic(){

    //Smart Dashboard Items
    SmartDashboard.putNumber("Launcher Velocity", getLauncherVelocity());
    SmartDashboard.putBoolean("At Set Velocity", isAtVelocity());
    SmartDashboard.putNumber("Launcher Setpoint", getVelocitySetpoint());
    SmartDashboard.putBoolean("Hood State", hoodSolenoid.get());
  }

  public void toggleHood(){
    hoodSolenoid.toggle();
  }

  public void hoodUp(){
    hoodSolenoid.set(false);
  }

  public void hoodDown(){
    hoodSolenoid.set(true);
  }

  public void manualLaunch(double speed){
    launcherLeader.set(speed);
  }

  public void velocityClosedLoopLaunch(){
    final double p = SmartDashboard.getNumber("P Gain", 0);
    final double i = SmartDashboard.getNumber("I Gain", 0);
    final double d = SmartDashboard.getNumber("D Gain", 0);
    final double iz = SmartDashboard.getNumber("I Zone", 0);
    final double ff = SmartDashboard.getNumber("Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Max Output", 0);
    final double min = SmartDashboard.getNumber("Min Output", 0);

      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { pidController.setP(p); kP = p; }
      if((i != kI)) { pidController.setI(i); kI = i; }
      if((d != kD)) { pidController.setD(d); kD = d; }
      if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
          pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; }
      pidController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocitySetpoint(double velocity){
        velocitySetpoint = velocity;
      }
    
      public double getVelocitySetpoint(){
        return velocitySetpoint;
      }
    
    
      public double getLauncherVelocity(){
        return launcherEncoder.getVelocity();
      }
    
      public boolean isAtVelocity(){
        double error = getLauncherVelocity() - velocitySetpoint;
        return (Math.abs(error) < allowableError);
      }
    


    
      public void StopShoot(){
          velocitySetpoint = 0;
          velocityClosedLoopLaunch();
      }
    
      public void longTarmacShoot(){
          hoodDown();
          velocitySetpoint = 5000;//4800
          velocityClosedLoopLaunch();
      }

      public void midTarmacShoot(){
        hoodDown();
        velocitySetpoint = 3700;//3500
        velocityClosedLoopLaunch();
    }
    
      public void fenderLowShoot(){
          hoodDown();
          velocitySetpoint = 2000;//3100
          velocityClosedLoopLaunch();
      }
    
      public void fenderHighShoot(){
        hoodUp();
        velocitySetpoint = 3500;
        velocityClosedLoopLaunch();
      }
    
      public void LShoot(){
        velocitySetpoint =3500;
        velocityClosedLoopLaunch();
      }
    
      public void ShortShoot(){
          velocitySetpoint = 2400;
          velocityClosedLoopLaunch();
      }
    
      public void startRPM() {
    
        velocityClosedLoopLaunch();
      }
    
      public void increaseRPM(){
        velocitySetpoint = velocitySetpoint + 10;
      }
    
      public void decreaseRPM(){
        velocitySetpoint = velocitySetpoint - 10;
      }
    
      public void autoSpeed(double yoffset){
        velocitySetpoint = 3.692*(yoffset*yoffset)-6.8485*(yoffset)+3288+20;
        velocityClosedLoopLaunch();
      }

      public void slowFlyWheel() {
        launcherLeader.set(.5);
      }

      public void fastFlyWheel() {
        launcherLeader.set(.85);
      }

      public  void stopFlyWheel() {
        launcherLeader.stopMotor();
        
      }

      public void toggleFlyWheel() {
        var tmp = getVelocitySetpoint();
        if (tmp > 0.0) {
          setVelocitySetpoint(0.0);
          lastSetpoint = tmp;
        } else if (lastSetpoint > 0.0) {
          setVelocitySetpoint(lastSetpoint);
          lastSetpoint = tmp;
        } else {
          setVelocitySetpoint(3400);
          lastSetpoint = 0.0;
        }
      }
    

    @Override
    public void simulationPeriodic(){


}


}
