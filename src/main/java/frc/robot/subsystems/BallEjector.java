package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.Constants.RIO_Channels_DIO;

public class BallEjector extends SubsystemBase {

    
    
    //Color Sensor Values
    DigitalInput redInput = new DigitalInput(RIO_Channels_DIO.COLOR_SENSOR_RED);
    DigitalInput blueInput = new DigitalInput(RIO_Channels_DIO.COLOR_SENSOR_BLUE);
    DigitalInput ejectorBeamBreak = new DigitalInput(RIO_Channels_DIO.EJECTOR_BEAM_BREAK);
    public String detectedBallColor = "None";
  

    CANSparkMax BallEject = new CANSparkMax(11, MotorType.kBrushless);


    RelativeEncoder ballejectEncoder;

    // private final static DriverStation ds = DriverStation;
    // private final static Alliance alliance = DriverStation.getAlliance();
    private static String alliance = "Not set";
    private double decayValue = 0.0;
    private double timeToDecay = 0.5; //seconds  assumes 10 ms loop timing may not be super accurate
    BallTower ejectorBallTower;
    

    public BallEjector(BallTower ballTower) {
        ejectorBallTower = ballTower;

        BallEject.restoreFactoryDefaults();
        BallEject.setSmartCurrentLimit(40);
        
        BallEject.setIdleMode(IdleMode.kBrake);

        //BallEject.setOpenLoopRampRate(.7);
        

        // alliance = ds.getAlliance();

        //m_colorMatcher.addColorMatch(Color.kBlue);
        //m_colorMatcher.addColorMatch(Color.kRed);

        BallEject.setInverted(true);
        // System.out.print("********************\n");
        // System.out.print(alliance.toString());
        // System.out.print("\n");
        // System.out.print(alliance.hashCode());
        // System.out.print("\n");
        // System.out.print(Color.kBlue);
        // System.out.print("\n");
        // System.out.print(Color.kRed);
        // System.out.print("********************\n");
    }

    public void ballUp() {
        BallEject.set(-0.50); // -1.0 and 1.0 apparently kills neo 550's
    }

    public void stop() {
        BallEject.stopMotor();
    }

    public void ballOut() {
        BallEject.set(0.75);
    }

    public void autoEject(){
        decayValue = decayValue - 0.01;

        if(isBallDetected() && !ejectorBallTower.isBallDetected()){
            decayValue = timeToDecay;
        } else if (isBallDetected() && ejectorBallTower.isBallDetected() && doesAllianceMatch()){
            decayValue = 0.0;
        }

        //ball detected
        if((isBallDetected() || (decayValue > 0.0 )) && !Climb.climbMode )
        {
        
            //correct color
            if(doesAllianceMatch() && !ejectorBallTower.isBallDetected())
            {   
                ballUp();
                //no ball in tower
                // if(!ejectorBallTower.isBallDetected())
                // {
                //     ballUp();
                // }
                // //ball in tower already hold ball
                // else
                // {
                //     stop();
                // }

            }
            //color does not match
            else if (!doesAllianceMatch())
            {
                ballOut();
            }
            else if (doesAllianceMatch() && ejectorBallTower.isBallDetected()){
                stop();
            }
        }
        else
        {
            stop();
        }
    }

    public boolean isRed()
    {
        return redInput.get();
    }

    
    public boolean isBlue()
    {
        return blueInput.get();
    }

    public boolean isBallDetectedColorSensor() //shows if a ball is detected by the color sensor
    {
         return (isBlue() || isRed());
    }

    public boolean isBallDetected(){  //shows if a ball is detected by the beam break
        return !ejectorBeamBreak.get();
    }

    public void determineBallColor(){
        if(isBallDetected() && isBallDetectedColorSensor()) //when both the color sensor and beam break detect a ball the color is determined
        {
            if(isBlue()){
                detectedBallColor = "Blue";
            }else if (isRed()){
                detectedBallColor = "Red";
            }else{
                detectedBallColor = "None";
            }
        }

        if(!isBallDetected()){
            detectedBallColor = "None";
        }
    }

    public boolean hasTwoBalls() {
        if (isBallDetected() && ejectorBallTower.isBallDetected()) {
                return true;
            } else {
                return false;
            }
        
    }

    public boolean doesAllianceMatch()
    {
        determineBallColor();
        if (alliance == "AutoOff"){
            return true;
        }
        else if (alliance == detectedBallColor){
            return true;
        }
        else{
            return false;
        }
    
        // if (alliance.toString() == "Red"){
        //     if(isRed()){
        //         return  true;
        //     }
        //     else{
        //         return false;
        //     }
        // }
        // else if (alliance.toString() == "Blue"){
        //     if(isBlue()){
        //         return true;
        //     }
        //     else{
        //         return false;
        //     }
        // }
        // else {
        //     return false;
        // }
    }

    



    // public Boolean checkBallColor()
    // {

    //     Color detectedColor = m_colorSensor.getColor();

        
    //     ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    //     if (alliance.toString() == "Blue") {
    //         if (match.color == Color.kBlue) {
    //             return true;
    //         } else {
    //             return false;
    //         }
    //     } else if (alliance.toString() == "Red") {
    //         if (match.color == Color.kRed) {
    //             return true;
    //         }
    //         else {
    //             return false;
    //         }
    //     } else {
    //         return false;
    //     }
       



    //}








    @Override
    public void periodic() {
        SmartDashboard.putString("Alliance Color", alliance.toString());
        SmartDashboard.putBoolean("Alliance Match", doesAllianceMatch());
        SmartDashboard.putBoolean("Ball Detected Beam Break", isBallDetected());
        SmartDashboard.putBoolean("Red", isRed());
        SmartDashboard.putBoolean("Blue", isBlue());
        SmartDashboard.putString("Detected Ball Color", detectedBallColor);
        //SmartDashboard.putNumber("Decay Value", decayValue);
        alliance = Robot.m_alliance;
        //SmartDashboard.putNumber("Eject Motor Current", BallEject.getOutputCurrent());
        //SmartDashboard.putBoolean("Ball Ejector ", value);





    }










}
