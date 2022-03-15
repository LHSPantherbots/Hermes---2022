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
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.*;

public class AutoCommand  extends SequentialCommandGroup {
    Trajectory redPickup_trajectory = new Trajectory();
    Trajectory redShoot_trajectory = new Trajectory();
    RamseteCommand ramseteCommand1;
    RamseteCommand ramseteCommand2; 
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
    public AutoCommand(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake, Conveyor conveyor, LimeLight limelight) {

    TrajectoryConfig trajectoryConfig_rev = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
    DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint)
        .addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint).setReversed(true);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
    DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint)
        .addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint);
    redPickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(
                new Translation2d(-0.75, 0)
            ), 
            new Pose2d(-1.5, 0, new Rotation2d()), trajectoryConfig_rev);
    redShoot_trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(-1.5, 0, new Rotation2d()),
        List.of(
            new Translation2d(-0.75, 0)
        ), 
        new Pose2d(-0.05, 0, new Rotation2d()), trajectoryConfig);
    
    Command waitForLauncher1 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher2 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher3 = new WaitForLauncherAtSpeed(launcher);
    Command waitForBeamBreak = new WaitForBeamBreak(ballTower);
    Command waitForShot1 = new WaitForShot(launcher, ballTower);
    Command waitForShot2 = new WaitForShot(launcher, ballTower);

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

    addCommands(
        new InstantCommand(() -> driveTrain.resetEncoders(), driveTrain),
        new InstantCommand(() -> driveTrain.zeroHeading(), driveTrain),
        new InstantCommand(() -> driveTrain.resetOdometry(redPickup_trajectory.getInitialPose()), driveTrain),
        new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
        new InstantCommand(conveyor::conveyerForward, conveyor),
        ramseteCommand1.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> intake.intakeRollersOff(), intake),
        new InstantCommand(() -> intake.intakeUp(), intake),
        ramseteCommand2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        // new InstantCommand(() -> limelight.ledOn(), limelight),
        new RunCommand(() -> limelight.ledPipeline(), limelight).withTimeout(.1),
        new RunCommand(() -> limelight.setPipeline(1), limelight).withTimeout(.1),
        new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(2).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        // new InstantCommand(() -> limelight.ledOff(), limelight),
        new RunCommand(() -> limelight.setPipeline(0), limelight).withTimeout(.1),
        new InstantCommand(() -> ballTower.liftBall(), ballTower),
        new InstantCommand(() -> launcher.autoMidTarmacShoot(), launcher),
        waitForLauncher1.withTimeout(.2),
        new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower).withTimeout(1),
        new InstantCommand(() -> ballTower.stopTower(), ballTower),
        // new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        // waitForShot1.withTimeout(1),
        new RunCommand(() -> ballTower.liftBall(), ballTower).withTimeout(2),
        // new InstantCommand(() -> ballTower.liftBall(), ballTower),
        // waitForBeamBreak.withTimeout(4),
        new InstantCommand(() -> ballTower.stopTower(), ballTower),
        //waitForBeamBreak,
        waitForLauncher2.withTimeout(.2),
        new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower).withTimeout(1),
        // new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        // waitForShot2.withTimeout(1),
        new InstantCommand(conveyor::stop, conveyor),
        new RunCommand(() -> launcher.setVelocitySetpoint(0), launcher).withTimeout(.1),
        waitForLauncher3
        );
        
    }
}
