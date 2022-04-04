// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  //public static boolean climbMode = false;
  public static String m_alliance = "None";
  public final Trajectories trajectories = new Trajectories();
  // Talon and Pigeon needed for subsystems defined here...
  public final static TalonSRX  talon1 = new TalonSRX(4);
  public final static PigeonIMU  pidgey = new PigeonIMU(talon1);
  // The robot's subsystems and commands are defined here...
  public final static ImuSubsystem imuSubsystem = new ImuSubsystem(pidgey);
  public final static DriveSubsystem driveTrain = new DriveSubsystem(imuSubsystem);
  public final static BallTower ballTower = new BallTower();
  public final static BallEjector ballEjector = new BallEjector(ballTower);
  public final Conveyor conveyor = new Conveyor();
  public final static Climb climb = new Climb();
  public final static ClimbPivot climbPivot = new ClimbPivot();
  public final static LimeLight limelight = new LimeLight();
  

  public final static Launcher launcher = new Launcher();  
  public final static Intake intake = new Intake(talon1);
  public final CompressorSubsystem compressorSubsystem = new CompressorSubsystem();
  public final static Leds leds = new Leds();
 // private final ClimbnHook climbnHook = new ClimbnHook();

  public final AutoCommand m_AutoCommand = new AutoCommand(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_ThreeBallAuto = new ThreeBallAuto(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_FourBallAuto = new FourBallAuto(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_AutoSmartTwoBall = new AutoSmartTwoBall(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_AutoSmartThreeBall = new AutoSmartThreeBall(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_AutoSmartFourBall = new AutoSmartFourBall(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_AutoSmartFiveBall = new AutoSmartFiveBall(driveTrain, launcher, ballTower, intake, conveyor, limelight);
  public final Command m_ArmUp = new ArmUp(driveTrain, climbPivot, climb); 
  public final Command m_AutoMidClimb = new AutoMidClimb(driveTrain, climbPivot, climb); 
  public final Command m_AutoHighClimb = new AutoHighClimb(driveTrain, climbPivot, climb);
  

  XboxController Gamepad0 = new XboxController(0);  //Driver Controller
  static XboxController Gamepad1 = new XboxController(1);  //Manipulator Controller
  
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

  public static SendableChooser<String> allianceChooser = new SendableChooser<>();
  public static SendableChooser<Command> autoChoice = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

//Populate Alliance Selection Chooser
    Shuffleboard.getTab("Autonomous").add(allianceChooser); 
    allianceChooser.addOption("Red", "Red");
    allianceChooser.addOption("Blue", "Blue");  
    allianceChooser.addOption("Auto Sort Off", "AutoOff");
    allianceChooser.setDefaultOption("Auto Sort Off", "AutoOff");

    Shuffleboard.getTab("Autonomous").add(autoChoice);
    autoChoice.addOption("Do Nothing", new RunCommand(()->driveTrain.teleopDrive(0, 0)));
    autoChoice.addOption("Two Ball Auto", m_AutoCommand);
    autoChoice.addOption("Three Ball Auto", m_ThreeBallAuto);
    autoChoice.addOption("Four Ball Auto", m_FourBallAuto);
    autoChoice.addOption("Smart Two Ball Auto", m_AutoSmartTwoBall);
    autoChoice.addOption("Smart Three Ball Auto", m_AutoSmartThreeBall);
    autoChoice.addOption("Smart Four Ball Auto", m_AutoSmartFourBall);
    autoChoice.addOption("Smart Five Ball", m_AutoSmartFiveBall);

    // Configure the button bindings

    configureButtonBindings();


    //Set Default States of Subsystems

    driveTrain.setDefaultCommand(
      new RunCommand(() -> driveTrain.teleopDrive(
              Gamepad0.getRawAxis(GamePadButtons.leftY)*-0.95, 
              Gamepad0.getRawAxis(GamePadButtons.rightX)*0.65,
              Gamepad0.getRightBumper()), driveTrain)

    );
    ballEjector.setDefaultCommand(
      new RunCommand(ballEjector::autoEject, ballEjector)
    );
    // intake.setDefaultCommand(new RunCommand(intake::stop, intake));

    leds.setDefaultCommand(
      new RunCommand(() -> leds.rainbow(), leds)
    );

   ballTower.setDefaultCommand(
     new RunCommand(ballTower::autoTower, ballTower)
     );

    conveyor.setDefaultCommand(
      new RunCommand(conveyor::conveyerStop, conveyor)
    );

    climb.setDefaultCommand(
      new RunCommand(() -> climb.manualClimb(
              Gamepad1.getRawAxis(GamePadButtons.leftY)*-0.95, 
             Gamepad1.getRawAxis(GamePadButtons.rightX)*0.65), climb)
     );


     climbPivot.setDefaultCommand(
       new RunCommand(climbPivot::defaultArmState, climbPivot)
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
    
    //Driver Gampad0

  
  // new JoystickButton(Gamepad0, GamePadButtons.Y)
  //     .whenPressed(new InstantCommand(limelight::ledPipeline, limelight))
  //     .whenPressed(new InstantCommand(()->limelight.setPipeline(1), limelight))
  //     .whenPressed(new RunCommand(limelight::startTakingSnapshots, limelight))
  //     .whileHeld(new RunCommand(driveTrain::limeLightAim, driveTrain))
  //     .whenReleased(new InstantCommand(limelight::stopTakingSnapshots, limelight))
  //     .whenReleased(new InstantCommand(() -> limelight.setPipeline(0), limelight));
    
    new JoystickButton(Gamepad0, GamePadButtons.LB)
      .whileHeld(new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower))
      // .whenReleased(new InstantCommand(ballTower::stopBelts));
      .whenReleased(new RunCommand(() -> ballTower.autoTower(), ballTower));

    new JoystickButton(Gamepad0, GamePadButtons.Y)
      .whenPressed(new InstantCommand(limelight::ledPipeline, limelight))
      .whenPressed(new InstantCommand(limelight::setPipelineOne, limelight))
      .whileHeld(new RunCommand(() -> driveTrain.limeLightAim(), driveTrain))
      .whenReleased(new InstantCommand(limelight::setPipelineZero, limelight));

        
    new POVButton(Gamepad0, GamePadButtons.Up)
      .whenPressed(new RunCommand(launcher::redShoot, launcher))
      .whenPressed(new RunCommand(leds::red, leds));

    
    new JoystickButton(Gamepad0, GamePadButtons.X)
      .whenPressed(new RunCommand(launcher::purpleShoot, launcher))
      .whenPressed(new RunCommand(leds::purple, leds));

    new JoystickButton(Gamepad0, GamePadButtons.A)
      .whenPressed(new RunCommand(launcher::greenShoot, launcher))
      .whenPressed(new RunCommand(leds::green, leds));
      
    new JoystickButton(Gamepad0, GamePadButtons.B)
      .whenPressed(new RunCommand(launcher::blueShoot, launcher))
      .whenPressed(new RunCommand(leds::blue, leds));
    
    new JoystickButton(Gamepad0, GamePadButtons.Select)
      .whenPressed(new RunCommand(launcher::stopFlyWheel, launcher))
      .whenReleased(new RunCommand(leds::rainbow, leds));



    //Manipulator Gamepad 1
    
    new JoystickButton(Gamepad1, GamePadButtons.LB)
      .whenHeld(new RunCommand(intake::intakeDown, intake))
      .whenReleased(new RunCommand(intake::intakeUp, intake));
    
    new JoystickButton(Gamepad1, GamePadButtons.X)
      .whenHeld(new RunCommand(intake::intakeRollersForward, intake))
      .whenHeld(new RunCommand(ballTower::liftBall, ballTower))
      .whenHeld(new RunCommand(conveyor::conveyerForward, conveyor))
      .whenReleased(new InstantCommand(intake::stop, intake))
      .whenReleased(new RunCommand(ballTower::autoTower, ballTower))
      .whenReleased(new InstantCommand(conveyor::stop, conveyor));

    new JoystickButton(Gamepad1, GamePadButtons.A)
      .whenHeld(new RunCommand(intake::intakeRollersReverse, intake))
      .whenReleased(new RunCommand (intake::intakeRollersOff, intake));
    
    new JoystickButton(Gamepad1, GamePadButtons.RB)
      .whenHeld(new RunCommand(climbPivot::armBack, climbPivot));

    new JoystickButton(Gamepad1, GamePadButtons.Start)
      .whenPressed(new RunCommand(conveyor::conveyerStop, conveyor))
      .whenPressed(new RunCommand(climb::setClimbModeFalse, climb))
      .whenReleased(new RunCommand(leds::rainbow, leds));

    new JoystickButton(Gamepad1, GamePadButtons.B)
      .whenHeld(new RunCommand(ballEjector::ballOut, ballEjector))
      .whenHeld(new RunCommand(ballTower::liftBall, ballTower))
      .whenReleased(new RunCommand(ballEjector::autoEject, ballEjector))
      .whenReleased(new RunCommand(ballTower::autoTower, ballTower));    

    new POVButton(Gamepad1, GamePadButtons.Down)
      .whenPressed(m_AutoMidClimb)
      .whenPressed(new RunCommand(launcher::stopFlyWheel, launcher))
      .whenPressed(new RunCommand(leds::bluePulse, leds));
    
    new POVButton(Gamepad1, GamePadButtons.Up)
      .whenPressed(m_ArmUp)
      .whenPressed(new RunCommand(launcher::stopFlyWheel, launcher))
      .whenPressed(new RunCommand(leds::bluePulse, leds));

    
    new POVButton(Gamepad1, GamePadButtons.Right)
      .whenPressed(m_AutoHighClimb)
      .whenPressed(new RunCommand(launcher::stopFlyWheel, launcher))
      .whenPressed(new RunCommand(leds::bluePulse, leds));


    new ManualClimbOverride()
      .whenActive(
        new RunCommand(() -> climb.manualClimb(
              Gamepad1.getRawAxis(GamePadButtons.leftY)*-0.95, 
              Gamepad1.getRawAxis(GamePadButtons.rightX)*0.65), climb))
      .whenActive(climb::setClimbModeFalse, climb)
      .whenActive(new RunCommand(leds::greenPulse, leds));
  
    
    
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // // An ExampleCommand will run in autonomous

    // var currTrajectory = trajectories.exampleTrajectory;
    // var currTrajectory = trajectory;

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
    // return twoBallAuto;
    return autoChoice.getSelected();
  }

//  static public void setClimbMode(){
//    climbMode=true;
//  }



}
