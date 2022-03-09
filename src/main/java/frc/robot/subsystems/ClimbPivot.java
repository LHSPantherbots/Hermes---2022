package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants.PH_Channel;


public class ClimbPivot extends SubsystemBase  {
  

    DoubleSolenoid armSolenoid = new DoubleSolenoid(26,PneumaticsModuleType.REVPH, PH_Channel.CLIMB_A, PH_Channel.CLIMB_B);



    public ClimbPivot() {
       

       
    }
    
    
    @Override
    public void periodic() {
        
    }

    public void defaultArmState() {
        //if (!Climb.climbMode){
            armSolenoid.set(DoubleSolenoid.Value.kForward);
        //}
    }

    public void armForward() {
        armSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void armBack() {
        armSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

  



  
}




