package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import frc.robot.commands.*;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DigitalInput;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class ColorSensor extends SubsystemBase{
    
    private final I2C.Port i2cPort = I2C.Port.kMXP;

    private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
  
    private final static Alliance alliance = DriverStation.getAlliance();


    DigitalInput redInput = new DigitalInput(8);
    DigitalInput blueInput = new DigitalInput(7);


    public ColorSensor(){

        m_colorMatcher.addColorMatch(Color.kBlue);
        m_colorMatcher.addColorMatch(Color.kRed);


    }
    /*
    public Boolean checkBallColor()
    {

        Color detectedColor = m_colorSensor.getColor();

        
        ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

        if (alliance.toString() == "Blue") {
            if (match.color == Color.kBlue) {
                return true;
            } else {
                return false;
            }
        } else if (alliance.toString() == "Red") {
            if (match.color == Color.kRed) {
                return true;
            }
            else {
                return false;
            }
        } else {
            return false;
        }

    }
    */
    @Override
    public void periodic() {
        //SmartDashboard.putBoolean("Ball Ejector ", value);
        SmartDashboard.putBoolean("Red", isRed());
        SmartDashboard.putBoolean("Blue", isBlue());
    



    }
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  
  
    public boolean isRed()
    {
      return redInput.get();
    }
  
    public boolean isBlue()
    {
      return blueInput.get();
    }
  






}
