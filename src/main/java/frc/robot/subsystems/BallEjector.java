package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.Robot;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
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

    
  

    CANSparkMax BallEject = new CANSparkMax(11, MotorType.kBrushless);


    RelativeEncoder ballejectEncoder;

    // private final static DriverStation ds = DriverStation;
    // private final static Alliance alliance = DriverStation.getAlliance();
    private static String alliance = "Red";
    private double decayValue = 0.0;
    private double timeToDecay = 0.5; //seconds  assumes 10 ms loop timing may not be super accurate
    BallTower ejectorBallTower;
    

    public BallEjector(BallTower ballTower) {
        ejectorBallTower = ballTower;

        BallEject.restoreFactoryDefaults();
        BallEject.setSmartCurrentLimit(20);
        
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
        BallEject.set(0.5);
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

    public boolean isBallDetected()
    {
        return (isBlue() || isRed());
    }

    public boolean hasTwoBalls() {
        if (isBallDetected()) {
            if (doesAllianceMatch() && ejectorBallTower.isBallDetected()) {
                return true;
            } else {
                return false;
            }
        } else {
            return false;
        }
    }

    public boolean doesAllianceMatch()
    {   
        if (alliance.toString() == "Red"){
            if(isRed()){
                return  true;
            }
            else{
                return false;
            }
        }
        else if (alliance.toString() == "Blue"){
            if(isBlue()){
                return true;
            }
            else{
                return false;
            }
        }
           else if (alliance.toString() == "AutoOff"){
               return true;
           }
         else {
             return false;
         }
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
        SmartDashboard.putBoolean("Ball Detected", isBallDetected());
        SmartDashboard.putBoolean("Red", isRed());
        SmartDashboard.putBoolean("Blue", isBlue());
        SmartDashboard.putNumber("Decay Value", decayValue);
        alliance = RobotContainer.allianceChooser.getSelected();
        SmartDashboard.putNumber("Eject Motor Current", BallEject.getOutputCurrent());
        //SmartDashboard.putBoolean("Ball Ejector ", value);





    }










}
