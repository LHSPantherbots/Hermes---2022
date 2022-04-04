// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBallAuto extends SequentialCommandGroup {
  /** Creates a new TwoBallAuto. */
  public TwoBallAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunCommand(RobotContainer.launcher::greenShoot, RobotContainer.launcher).withTimeout(2),
      new RunCommand(RobotContainer.ballTower::feedBallToLauncher, RobotContainer.ballTower).withTimeout(2),
      new ParallelCommandGroup(
      new RunCommand(RobotContainer.intake::intakeDownnRoll, RobotContainer.intake),
      new RunCommand(() -> RobotContainer.driveTrain.teleopDrive(-.4, 0), RobotContainer.driveTrain)
      ).withTimeout(3),
      new InstantCommand(RobotContainer.intake::intakeUp, RobotContainer.intake),
      new RunCommand(() -> RobotContainer.driveTrain.teleopDrive(.4, 0), RobotContainer.driveTrain).withTimeout(1),
      new RunCommand(RobotContainer.ballTower::autoTower, RobotContainer.ballTower).withTimeout(2),
      new RunCommand(RobotContainer.launcher::greenShoot, RobotContainer.launcher).withTimeout(2),
      new RunCommand(RobotContainer.ballTower::feedBallToLauncher, RobotContainer.ballTower).withTimeout(2)
      );
  }
}
