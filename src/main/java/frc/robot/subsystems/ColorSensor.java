package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

public class ColorSensor extends SubsystemBase{
      
    // private final static Alliance alliance = DriverStation.getAlliance();


    DigitalInput redInput = new DigitalInput(8);
    DigitalInput blueInput = new DigitalInput(7);


    public ColorSensor(){


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
        //SmartDashboard.putBoolean("Red", isRed());
        //SmartDashboard.putBoolean("Blue", isBlue());
    



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
