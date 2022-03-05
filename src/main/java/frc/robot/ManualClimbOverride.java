// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants.GamePadButtons;

/** Add your docs here. */
public class ManualClimbOverride extends Button{


    public ManualClimbOverride(){
    }



    public boolean get(){
        if ((Math.abs(RobotContainer.Gamepad1.getRawAxis(GamePadButtons.leftY))>0.2) 
            || 
            (Math.abs(RobotContainer.Gamepad1.getRawAxis(GamePadButtons.rightX))>0.2)
            )
        {
            return true;
        }
        else
        {
            return false;
        }
    }





}
