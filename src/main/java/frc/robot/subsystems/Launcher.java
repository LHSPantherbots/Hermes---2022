package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CAN_ID;
import frc.robot.Constants.PH_Channel;


public class Launcher extends SubsystemBase{
    
    CANSparkMax launcherLeader = new CANSparkMax(CAN_ID.LAUNCH_LEADER, MotorType.kBrushless);
    CANSparkMax launcherFollower = new CANSparkMax(CAN_ID.LAUNCH_FOLLOWER, MotorType.kBrushless);
    CANSparkMax topRoller = new CANSparkMax(CAN_ID.LAUNCH_TOPROLLER, MotorType.kBrushless);

    RelativeEncoder launcherEncoder;
    RelativeEncoder tRollerEncoder;

    Solenoid hoodSolenoid = new Solenoid(26, PneumaticsModuleType.REVPH, PH_Channel.HOOD);

    
    
   

    private SparkMaxPIDController pidController;
    private SparkMaxPIDController pidControllerTop;
   public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, allowableError;
   private double velocitySetpoint = 0.0;
   private double lastSetpoint = 3500;


   public double kPtop, kItop, kDtop, kIztop, kFFtop, kMaxOutputTop, kMinOutputTop, maxRPMtop, allowableErrortop;
   private double velocitySetpointTop = 0.0;
   private double lastSetpointTop = 3500;
    

    public Launcher() {


//sets up follower and makes the direction inverted to the leader
launcherLeader.restoreFactoryDefaults();

launcherFollower.restoreFactoryDefaults();

topRoller.restoreFactoryDefaults();

launcherLeader.setSmartCurrentLimit(60);
launcherFollower.setSmartCurrentLimit(60);
topRoller.setSmartCurrentLimit(60);

launcherLeader.setClosedLoopRampRate(.25);
launcherFollower.setClosedLoopRampRate(.25);

topRoller.setClosedLoopRampRate(.25);

launcherLeader.setIdleMode(IdleMode.kCoast);
launcherFollower.setIdleMode(IdleMode.kCoast);

topRoller.setIdleMode(IdleMode.kCoast);
 //sets up follower and makes the direction inverted to the leader
launcherFollower.follow(launcherLeader, true);

topRoller.setInverted(true);

 //Sets up endcoders
 launcherEncoder = launcherLeader.getEncoder();

 tRollerEncoder = topRoller.getEncoder();

 pidController = launcherLeader.getPIDController();

 pidControllerTop = topRoller.getPIDController();

 // PID coefficients these will need to be tuned
 kP = 0.00015;//0.00025; //5e-5; 
 kI =  0;//1e-6;
 kD = 0.0008;//0.0004; 
 kIz = 0;
 kFF = 0.00017;//0.00019;
 kMaxOutput = 1; 
 kMinOutput = -1;
 maxRPM = 5700;
 allowableError = 100; // 50    //Lets the system known when the velocity is close enough to launch

//  Top Roller PID coefficients these will need to be tuned 
kPtop = 0.0001;//0.00025; //5e-5; 
kItop =  0;//1e-6;
kDtop = 0.0;//0.0004; 
kIztop = 0; 
kFFtop = 0.00018;//0.00019
kMaxOutputTop = 1; 
kMinOutputTop = -1;
maxRPMtop = 5700;
allowableErrortop = 100; // 50   //Lets the system known when the velocity is close enough to launch







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

// set Top Roller PID coefficients
pidControllerTop.setP(kPtop);
pidControllerTop.setI(kItop);
pidControllerTop.setD(kDtop);
pidControllerTop.setIZone(kIztop);
pidControllerTop.setFF(kFFtop);
pidControllerTop.setOutputRange(kMinOutputTop, kMaxOutputTop);

SmartDashboard.putNumber("P Gain TopR", kPtop);
SmartDashboard.putNumber("I Gain TopR", kItop);
SmartDashboard.putNumber("D Gain TopR", kDtop);
SmartDashboard.putNumber("I Zone TopR", kIztop);
SmartDashboard.putNumber("Feed Forward TopR", kFFtop);
SmartDashboard.putNumber("Max Output Topr", kMaxOutputTop);
SmartDashboard.putNumber("Min Output TopR", kMinOutputTop);





    }

    @Override
    public void periodic(){

    //Smart Dashboard Items
    SmartDashboard.putNumber("Launcher Velocity", getLauncherVelocity());
    SmartDashboard.putBoolean("At Set Velocity", isAtVelocity());
    SmartDashboard.putNumber("Launcher Setpoint", getVelocitySetpoint());
    SmartDashboard.putBoolean("Hood State", hoodSolenoid.get());


    SmartDashboard.putNumber("Launcher Velocity TopR", getTopRollerVelocity());
    SmartDashboard.putBoolean("At Set Velocity TopR", isAtVelocityTop());
    SmartDashboard.putNumber("Launcher Setpoint TopR", getVelocitySetpointTop());

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

  public void velocityClosedLoopLaunch2(){
    final double p = SmartDashboard.getNumber("P Gain", 0);
    final double i = SmartDashboard.getNumber("I Gain", 0);
    final double d = SmartDashboard.getNumber("D Gain", 0);
    final double iz = SmartDashboard.getNumber("I Zone", 0);
    final double ff = SmartDashboard.getNumber("Feed Forward", 0);
    final double max = SmartDashboard.getNumber("Max Output", 0);
    final double min = SmartDashboard.getNumber("Min Output", 0);

    final double ptop = SmartDashboard.getNumber("P Gain TopR", 0);
    final double itop = SmartDashboard.getNumber("I Gain TopR", 0);
    final double dtop = SmartDashboard.getNumber("D Gain TopR", 0);
    final double iztop = SmartDashboard.getNumber("I Zone TopR", 0);
    final double fftop = SmartDashboard.getNumber("Feed Forward TopR", 0);
    final double maxtop = SmartDashboard.getNumber("Max Output TopR", 0);
    final double mintop = SmartDashboard.getNumber("Min Output TopR", 0);


      // if PID coefficients on SmartDashboard have changed, write new values to controller
      if((p != kP)) { pidController.setP(p); kP = p; }
      if((i != kI)) { pidController.setI(i); kI = i; }
      if((d != kD)) { pidController.setD(d); kD = d; }
      if((iz != kIz)) { pidController.setIZone(iz); kIz = iz; }
      if((ff != kFF)) { pidController.setFF(ff); kFF = ff; }
      if((max != kMaxOutput) || (min != kMinOutput)) { 
          pidController.setOutputRange(min, max); 
            kMinOutput = min; kMaxOutput = max; }
      
      if((ptop != kPtop)) { pidControllerTop.setP(ptop); kPtop = ptop; }
      if((itop != kItop)) { pidControllerTop.setI(itop); kItop = itop; }
      if((dtop != kDtop)) { pidControllerTop.setD(dtop); kDtop = dtop; }
      if((iztop != kIztop)) { pidControllerTop.setIZone(iztop); kIztop = iztop; }
      if((fftop != kFFtop)) { pidControllerTop.setFF(fftop); kFFtop = fftop; }
      if((maxtop != kMaxOutputTop) || (mintop != kMinOutputTop)) { 
          pidControllerTop.setOutputRange(mintop, maxtop); 
             kMinOutputTop = mintop; kMaxOutputTop = maxtop; }
      pidController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
      pidControllerTop.setReference(velocitySetpointTop, CANSparkMax.ControlType.kVelocity);
    }

    public void setVelocitySetpoint(double velocity){
        velocitySetpoint = velocity;
      }

    public void setVelocitySetPointTop(double topvelocity){
        velocitySetpointTop = topvelocity;
    }
    
      public double getVelocitySetpoint(){
        return velocitySetpoint;
      }
    
      public double getVelocitySetpointTop(){
        return velocitySetpointTop;
      }

    
      public double getLauncherVelocity(){
        return launcherEncoder.getVelocity();
      }

      public double getTopRollerVelocity(){
        return tRollerEncoder.getVelocity();
      }
    
      public boolean isAtVelocity(){
        double error = getLauncherVelocity() - velocitySetpoint;
        return (Math.abs(error) < allowableError);
      }
    
      public boolean isAtVelocityTop(){
        double error = getTopRollerVelocity() - velocitySetpointTop;
        return (Math.abs(error) < allowableError);
      }

    
      public void StopShoot(){
          velocitySetpoint = 0;
          velocitySetpointTop = 0;
          velocityClosedLoopLaunch();
      }
    
      public void longTarmacShoot(){
          hoodDown();
          velocitySetpoint = 4000;//4800
          velocitySetpointTop = 5500;
          velocityClosedLoopLaunch();
      }

      public void midTarmacShoot(){
        //hoodDown();
        velocitySetpoint = 2500;//2250      //3500 ---- 3600 3/5/22
        velocitySetpointTop = 3750; //3500             //3500     Remenber : 3900
        velocityClosedLoopLaunch();
    }

    public void autoMidTarmacShoot(){
      //hoodDown();
      velocitySetpoint = 2500; //2250           //3500 ---- 3600 3/5/22
      velocitySetpointTop = 3750; //3500
      velocityClosedLoopLaunch();
  }
    
      public void fenderLowShoot(){
          //hoodDown();
          velocitySetpoint = 1000;//3100
          velocitySetpointTop = 2500; //2000
          velocityClosedLoopLaunch();
      }
    
      public void fenderHighShoot(){
        //hoodUp();
        velocitySetpoint = 2125;
        velocitySetpointTop = 3187.5;
        velocityClosedLoopLaunch();
      }
    
      public void LShoot(){
        velocitySetpoint = 3500;
        velocitySetpointTop = 3500;
        velocityClosedLoopLaunch();
      }
    
      public void ShortShoot(){
          velocitySetpoint = 1000;
          velocitySetpointTop = 2400;
          velocityClosedLoopLaunch();
      }
    
      public void startRPM() {
    
        velocityClosedLoopLaunch();
      }
    
      public void increaseRPM(){
        velocitySetpoint = velocitySetpoint + 10;
        velocitySetpointTop = velocitySetpointTop + 10;
      }
    
      public void decreaseRPM(){
        velocitySetpoint = velocitySetpoint - 10;
        velocitySetpointTop = velocitySetpointTop - 10;
      }
    
      public void autoSpeed(double yoffset){
        velocitySetpoint = 3.692*(yoffset*yoffset)-6.8485*(yoffset)+3288+20;
        velocitySetpointTop = 3.692*(yoffset*yoffset)-6.8485*(yoffset)+3288+20;
        velocityClosedLoopLaunch();
      }

      public void slowFlyWheel() {
        launcherLeader.set(.5);
        topRoller.set(.5);
      }

      public void fastFlyWheel() {
        launcherLeader.set(.85);
        topRoller.set(.85);
      }

      public void stopFlyWheel() {
        launcherLeader.stopMotor();
        topRoller.stopMotor();
        
      }

      public void toggleFlyWheel() {
        var tmp = getVelocitySetpoint();
        var tmpTop = getVelocitySetpointTop();
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
        if (tmpTop > 0.0){
          setVelocitySetPointTop(0.0);
          lastSetpointTop = tmpTop;
        } else if (lastSetpointTop > 0.0){
          setVelocitySetPointTop(lastSetpointTop);
          lastSetpointTop = tmpTop;
        }else {
          setVelocitySetPointTop(0);
          lastSetpointTop = 0.0;
        }

          



      }
    

    @Override
    public void simulationPeriodic(){


}


public void velocityClosedLoopLaunch(){
   pidController.setP(kP);
   pidController.setI(kI);
   pidController.setD(kD);
   pidController.setIZone(kI);
   pidController.setFF(kFF);
   pidController.setOutputRange(kMinOutput, kMaxOutput); 
   
   
    pidControllerTop.setP(kPtop);
    pidControllerTop.setI(kItop);
    pidControllerTop.setD(kDtop);
    pidControllerTop.setIZone(kIztop);
    pidControllerTop.setFF(kFFtop);
    pidControllerTop.setOutputRange(kMinOutputTop, kMaxOutputTop); 
  
    pidController.setReference(velocitySetpoint, CANSparkMax.ControlType.kVelocity);
    pidControllerTop.setReference(velocitySetpointTop, CANSparkMax.ControlType.kVelocity);
  }



}
