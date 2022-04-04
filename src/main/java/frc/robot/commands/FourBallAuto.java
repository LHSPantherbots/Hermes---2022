package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.*;

public class FourBallAuto  extends SequentialCommandGroup {
    Trajectory redPickup_trajectory = new Trajectory();
    Trajectory redShoot_trajectory = new Trajectory();
    Trajectory intakeDown_trajectory = new Trajectory();
    Trajectory forwardShoot2_trajectory = new Trajectory();
    RamseteCommand ramseteCommand1;
    RamseteCommand ramseteCommand2; 
    RamseteCommand ramseteCommand3;
    Command waitForLauncher1; 
    Command waitForLauncher2; 
    Command waitForLauncher3;
    Command waitForBeamBreak;
    Command waitForShot1;
    Command waitForShot2;
    private final PIDController left_PidController = new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    private final PIDController right_PidController =new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    public DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
        DriveTrainConstants.kDriveKinematics, 10);

    public DifferentialDriveKinematicsConstraint ddKinematicConstraint = new DifferentialDriveKinematicsConstraint(DriveTrainConstants.kDriveKinematics, DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DriveTrainConstants.kMaxSpeedMetersPerSecond);
    public FourBallAuto(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake, Conveyor conveyor, LimeLight limelight) {

    TrajectoryConfig trajectoryConfig_rev = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
    DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint)
        .addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint).setReversed(true);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
    DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint)
        .addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint);
    redPickup_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
        List.of(
            new Translation2d(Units.inchesToMeters(-12.75), 0)
        ), 
        new Pose2d(Units.inchesToMeters(-63), 0, new Rotation2d()), trajectoryConfig_rev);
    redShoot_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(-63), 0, new Rotation2d(Units.degreesToRadians(9))),
        List.of(
            new Translation2d(Units.inchesToMeters(-100), Units.inchesToMeters(0))
        ), 
        new Pose2d(Units.inchesToMeters(-180), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))), trajectoryConfig_rev);
        intakeDown_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d()),
            List.of(
                new Translation2d(Units.inchesToMeters(2), Units.inchesToMeters(0))
            ), 
            new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))), trajectoryConfig);
        forwardShoot2_trajectory = TrajectoryGenerator.generateTrajectory(
             new Pose2d(Units.inchesToMeters(-180), 0, new Rotation2d(0)),
            List.of(
                  new Translation2d(Units.inchesToMeters(-80), Units.inchesToMeters(0))
             ), 
            new Pose2d(Units.inchesToMeters(-63), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))), trajectoryConfig);


    Command waitForLauncher1 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher2 = new WaitForLauncherAtSpeed(launcher);

    RamseteCommand ramseteCommand1 = new RamseteCommand(
        redPickup_trajectory,
        driveTrain::getPose,
        new RamseteController(
            DriveTrainConstants.kRamseteB,
            DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        driveTrain::getWheelSpeeds,
        left_PidController,
        right_PidController,
        driveTrain::tankDriveVolts,
        driveTrain
    );

    RamseteCommand ramseteCommand2 = new RamseteCommand(
        redShoot_trajectory,
        driveTrain::getPose,
        new RamseteController(
            DriveTrainConstants.kRamseteB,
            DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        driveTrain::getWheelSpeeds,
        left_PidController,
        right_PidController,
        driveTrain::tankDriveVolts,
        driveTrain
    );


    RamseteCommand ramseteCommand0 = new RamseteCommand(
        intakeDown_trajectory,
        driveTrain::getPose,
        new RamseteController(
            DriveTrainConstants.kRamseteB,
            DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        driveTrain::getWheelSpeeds,
        left_PidController,
        right_PidController,
        driveTrain::tankDriveVolts,
        driveTrain
    );


    RamseteCommand ramseteCommand3 = new RamseteCommand(
        forwardShoot2_trajectory,
        driveTrain::getPose,
        new RamseteController(
            DriveTrainConstants.kRamseteB,
            DriveTrainConstants.kRamseteZeta),
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter),
        DriveTrainConstants.kDriveKinematics,
        driveTrain::getWheelSpeeds,
        left_PidController,
        right_PidController,
        driveTrain::tankDriveVolts,
        driveTrain
    );

    addCommands(
        new InstantCommand(() -> driveTrain.resetEncoders(), driveTrain),
        new InstantCommand(() -> driveTrain.zeroHeading(), driveTrain),
        new InstantCommand(() -> driveTrain.resetOdometry(redPickup_trajectory.getInitialPose()), driveTrain),
        new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
        ramseteCommand0.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(conveyor::conveyerForward, conveyor),
        ramseteCommand1.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> intake.intakeRollersOff(), intake),
        new InstantCommand(() -> intake.intakeUp(), intake),
        // ramseteCommand2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        // new InstantCommand(() -> limelight.ledOn(), limelight),
        // new RunCommand(() -> limelight.ledPipeline(), limelight).withTimeout(.1),
        new InstantCommand(limelight::ledPipeline, limelight),
        new InstantCommand(() -> limelight.setPipeline(1), limelight),
        new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(1.5).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        // new InstantCommand(() -> limelight.ledOff(), limelight),
        // new RunCommand(() -> limelight.setPipeline(0), limelight).withTimeout(.1),
        new InstantCommand(() -> ballTower.liftBall(), ballTower),
        new InstantCommand(() -> launcher.purpleShoot(), launcher),
        waitForLauncher1.withTimeout(.2),
        new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower).withTimeout(1),
        new InstantCommand(() -> ballTower.stopTower(), ballTower), 
        new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
        ramseteCommand2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
       
        new WaitCommand(3),
        ramseteCommand3.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(limelight::ledPipeline, limelight),
        new InstantCommand(() -> limelight.setPipeline(1), limelight),
        new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(1.5).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        // new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        // waitForShot1.withTimeout(1),
        new RunCommand(() -> ballTower.liftBall(), ballTower).withTimeout(1),
        new InstantCommand(() -> launcher.purpleShoot(), launcher),
        // new InstantCommand(() -> ballTower.liftBall(), ballTower),
        // waitForBeamBreak.withTimeout(4),
        
        //waitForBeamBreak,
        waitForLauncher2.withTimeout(.2),
        new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower).withTimeout(1),
        // new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        // waitForShot2.withTimeout(1),
        new InstantCommand(() -> ballTower.stopTower(), ballTower),
        new InstantCommand(conveyor::stop, conveyor),
        new RunCommand(() -> launcher.setVelocitySetpoint(0), launcher).withTimeout(.1),
        // waitForLauncher3,
        new RunCommand(() -> limelight.setPipeline(0), limelight).withTimeout(.1)
        );
        
    }
}
