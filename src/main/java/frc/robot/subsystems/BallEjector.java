package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.*;
import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.RIO_Channels_DIO;

public class BallEjector extends SubsystemBase {

    
    
    //Color Sensor Values
    DigitalInput redInput = new DigitalInput(RIO_Channels_DIO.COLOR_SENSOR_RED);
    DigitalInput blueInput = new DigitalInput(RIO_Channels_DIO.COLOR_SENSOR_BLUE);

    
  

    CANSparkMax BallEject = new CANSparkMax(11, MotorType.kBrushless);

    RelativeEncoder ballejectEncoder;

    // private final static DriverStation ds = DriverStation;
    // private final static Alliance alliance = DriverStation.getAlliance();
    private final static String alliance = "Blue";
    BallTower ejectorBallTower;
    

    public BallEjector(BallTower ballTower) {
        ejectorBallTower = ballTower;

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
        BallEject.set(-.5);
    }

    public void stop() {
        BallEject.stopMotor();
    }

    public void ballOut() {
        BallEject.set(.5);
    }

    public void autoEject(){
        //ball detected
        if(isBallDetected() || !RobotContainer.climbMode)
        {
            //correct color
            if(doesAllianceMatch())
            {
                //no ball in tower
                if(!ejectorBallTower.isBallDetected())
                {
                    ballUp();
                }
                //ball in tower already hold ball
                else
                {
                    stop();
                }

            }
            //color does not match
            else 
            {
                ballOut();
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
        //SmartDashboard.putBoolean("Ball Ejector ", value);





    }










}
