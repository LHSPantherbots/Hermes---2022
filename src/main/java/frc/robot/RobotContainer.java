// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  public final Trajectories trajectories = new Trajectories();
  // Talon and Pigeon needed for subsystems defined here...
  public final static TalonSRX  talon1 = new TalonSRX(4);
  public final static PigeonIMU  pidgey = new PigeonIMU(talon1);
  // The robot's subsystems and commands are defined here...
  public final static ImuSubsystem imuSubsystem = new ImuSubsystem(pidgey);
  public final static DriveSubsystem driveTrain = new DriveSubsystem(imuSubsystem);
  public final static BallTower ballTower = new BallTower();
  public final BallEjector ballEjector = new BallEjector(ballTower);
  public final Conveyor conveyor = new Conveyor();
  public final Climb climb = new Climb();
  public final ClimbPivot climbPivot = new ClimbPivot();


  public final static Launcher launcher = new Launcher();  
  public final static Intake intake = new Intake(talon1);
  public final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
  public final Leds leds = new Leds();
 // private final ClimbnHook climbnHook = new ClimbnHook();

  public final AutoCommand m_AutoCommand = new AutoCommand(driveTrain);

  XboxController Gamepad0 = new XboxController(0);  //Driver Controller
  XboxController Gamepad1 = new XboxController(1);  //Manipulator Controller


  // PIDControllers for Path Following defined here to avoid new PIDControlers being created each time auto is enabled in testing
  private final PIDController left_PidController = new PIDController(DriveTrainConstants.left_Kp, 0, 1.2);
  private final PIDController right_PidController =new PIDController(DriveTrainConstants.right_Kp, 0, 1.2);
  private final Command twoBallAuto = new TwoBallAuto();
  
  SequentialCommandGroup stopIntake = new SequentialCommandGroup(
                new RunCommand(() -> intake.stop(), intake), 
                new InstantCommand(() -> intake.intakeUp(), intake));
  ParallelCommandGroup ballUp = new ParallelCommandGroup(
                new RunCommand(() -> ballTower.liftBall(), ballTower), 
                new RunCommand(() -> ballEjector.ballUp(), ballEjector), 
                new RunCommand(() -> intake.run(), intake));
  ParallelCommandGroup ballDown = new ParallelCommandGroup(
                new RunCommand(() -> ballTower.lowerBall(), ballTower), 
                new RunCommand(() -> ballEjector.ballOut(), ballEjector), 
                new RunCommand(() -> conveyor.conveyerBackward(), conveyor));
  ParallelCommandGroup ballStop = new ParallelCommandGroup(
                new RunCommand(() -> ballTower.stopTower(), ballTower), 
                new RunCommand(() -> ballEjector.stop(), ballEjector), 
                new RunCommand(() -> intake.stop(), intake));
  ParallelCommandGroup ejectBall = new ParallelCommandGroup(
                new RunCommand(() -> ballEjector.ballOut(), ballEjector), 
                new RunCommand(() -> conveyor.conveyerBackward(), conveyor));
  ParallelCommandGroup ejectStop = new ParallelCommandGroup(
                new RunCommand(() -> ballEjector.stop(), ballEjector), 
                new RunCommand(() -> conveyor.conveyerStop(), conveyor));
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings

    
    configureButtonBindings();


    //Set Default States of Subsystems

    driveTrain.setDefaultCommand(
      new RunCommand(() -> driveTrain.teleopDrive(
              Gamepad0.getRawAxis(GamePadButtons.leftY)*-0.95, 
              Gamepad0.getRawAxis(GamePadButtons.rightX)*0.65), driveTrain)

    );
    ballEjector.setDefaultCommand(
      new RunCommand(ballEjector::autoEject, ballEjector)
    );
    //intake.setDefaultCommand(new RunCommand(intake::intakeUp, intake));

    leds.setDefaultCommand(
      new RunCommand(() -> leds.rainbow(), leds)
    );

    ballTower.setDefaultCommand(
      new RunCommand(ballTower::autoTower, ballTower)
    );

    conveyor.setDefaultCommand(
      new RunCommand(conveyor::conveyerForward, conveyor)
    );

     climb.setDefaultCommand(
      new RunCommand(() -> climb.manualClimb(
              Gamepad1.getRawAxis(GamePadButtons.leftY)*-0.95, 
             Gamepad1.getRawAxis(GamePadButtons.rightX)*0.65), climb)
     );


     climbPivot.setDefaultCommand(
       new RunCommand(climbPivot::armForward, climbPivot)
     );
    // ballTower.setDefaultCommand(
    //   new RunCommand(() -> ballTower.feedBallToLauncher(Gamepad0.getRawAxis(GamePadButtons.rightTrigger)), ballTower)
    // );

  }

  public CANSparkMax getLeftSparkMax() {
    return driveTrain.getLeftCanSparkMax();
  }

  public CANSparkMax getRightSparkMax() {
    return driveTrain.getRightCanSparkMax();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Combining whileHeld and whenReleased into one button object
    // new JoystickButton(Gamepad0, GamePadButtons.A)
    //   .whileHeld(new RunCommand(() -> launcher.slowFlyWheel(), launcher))
    //   .whenReleased(new RunCommand(() -> launcher.stopFlyWheel(), launcher));
    
    
    // new JoystickButton(Gamepad0, GamePadButtons.B)
    //   .whileHeld(new RunCommand(() -> launcher.fastFlyWheel(), launcher))
    //   .whenReleased(new RunCommand(() -> launcher.stopFlyWheel(), launcher));
    
    //Driver Gampad0

   
    new JoystickButton(Gamepad0, GamePadButtons.RB)
      .whileHeld(ballUp)
      .whenReleased(ballStop);
    
    new JoystickButton(Gamepad0, GamePadButtons.Y)
      .whenPressed(launcher::hoodUp, launcher);

    new JoystickButton(Gamepad0, GamePadButtons.B)
      .whenPressed(launcher::hoodDown, launcher);
    
    new JoystickButton(Gamepad0, GamePadButtons.LB)
      .whileHeld(new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower))
      .whenReleased(new InstantCommand(ballTower::stopBelts));

    new JoystickButton(Gamepad0, GamePadButtons.X)
      .whenPressed(new InstantCommand(launcher::increaseRPM, launcher))
      .whenReleased(new RunCommand(launcher::startRPM, launcher));

    new JoystickButton(Gamepad0, GamePadButtons.A)
    .whenPressed(new InstantCommand(launcher::decreaseRPM, launcher))
    .whenReleased(new RunCommand(launcher::startRPM, launcher));

    
    new POVButton(Gamepad0, GamePadButtons.Up)
      .whenPressed(new RunCommand(launcher::longTarmacShoot, launcher))
      .whenPressed(new RunCommand(leds::red, leds));

      new POVButton(Gamepad0, GamePadButtons.Left)
      .whenPressed(new RunCommand(launcher::midTarmacShoot, launcher))
      .whenPressed(new RunCommand(leds::purple, leds));

    new POVButton(Gamepad0, GamePadButtons.Down)
      .whenPressed(new RunCommand(launcher::fenderHighShoot, launcher))
      .whenPressed(new RunCommand(leds::green, leds));
      
    new POVButton(Gamepad0, GamePadButtons.Right)
      .whenPressed(new RunCommand(launcher::fenderLowShoot, launcher))
      .whenPressed(new RunCommand(leds::blue, leds));
    
    new JoystickButton(Gamepad0, GamePadButtons.Select)
      .whenPressed(new RunCommand(launcher::stopFlyWheel, launcher))
      .whenReleased(new RunCommand(leds::rainbow, leds));



    //Manipulator Gamepad
    

    new JoystickButton(Gamepad1, GamePadButtons.LB)
        .whenHeld(new RunCommand(intake::intakeDown, intake))
        .whenReleased(new RunCommand(intake::intakeUp, intake));
    
    new JoystickButton(Gamepad1, GamePadButtons.X)
        .whenHeld(new RunCommand(intake::intakeRollersForward, intake))
        .whenReleased(new RunCommand (intake::intakeRollersOff, intake));

    new JoystickButton(Gamepad1, GamePadButtons.A)
        .whenHeld(new RunCommand(intake::intakeRollersReverse, intake))
        .whenReleased(new RunCommand (intake::intakeRollersOff, intake));
    
     new JoystickButton(Gamepad1, GamePadButtons.RB)
     .whenHeld(new RunCommand(climbPivot::armBack, climbPivot));
     
    
    
    }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // An ExampleCommand will run in autonomous

    // var currTrajectory = trajectories.exampleTrajectory;

    // // Set bot known position to be the same as the first position of the trajectory
    // driveTrain.resetOdometry(currTrajectory.getInitialPose());

    // SmartDashboard.putNumber("Trajectory Durration", currTrajectory.getTotalTimeSeconds());

    // RamseteCommand ramseteCommand = new RamseteCommand(
    //   currTrajectory,
    //   driveTrain::getPose,
    //   new RamseteController(
    //     DriveTrainConstants.kRamseteB,
    //     DriveTrainConstants.kRamseteZeta),
    //   new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
    //   DriveTrainConstants.kDriveKinematics,
    //   driveTrain::getWheelSpeeds,
    //   left_PidController,
    //   right_PidController,
    //   driveTrain::tankDriveVolts,
    //   driveTrain
    // );

    // return ramseteCommand.andThen(() -> driveTrain.tankDriveVolts(0, 0));
    return twoBallAuto;
  }
}
