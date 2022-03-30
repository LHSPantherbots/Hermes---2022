package frc.robot.commands;

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
import java.util.List;

public class AutoSmartFiveBall extends SequentialCommandGroup {
    Trajectory first_Pickup_trajectory = new Trajectory();
    Trajectory first_Shoot_trajectory = new Trajectory();
    Trajectory second_Pickup_trajectory = new Trajectory();
    Trajectory second_Shoot_trajectory = new Trajectory();
    Trajectory third_Pickup_Trajectory = new Trajectory();
    Trajectory third_Shoot_Trajectory = new Trajectory();
    RamseteCommand ramseteCommand_first_pickup;
    RamseteCommand ramseteCommand_first_shoot;
    RamseteCommand ramseteCommand_second_pickup;
    RamseteCommand ramseteCommand_second_shoot;
    RamseteCommand ramseteCommand_third_pickup;
    RamseteCommand ramseteCommand_third_shoot;
    Command autoSmartShot1; 
    Command autoSmartShot2; 
    Command autoSmartShot3;
    Command autoSmartShot4;
    Command autoSmartShot5;
    Command autoSmartTower1;
    Command autoSmartTower2;
    Command autoSmartTower3;
    Command autoSmartTower4;
    private final PIDController left_PidController = new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    private final PIDController right_PidController =new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    public DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
        DriveTrainConstants.kDriveKinematics, 10);

    public DifferentialDriveKinematicsConstraint ddKinematicConstraint = new DifferentialDriveKinematicsConstraint(DriveTrainConstants.kDriveKinematics, DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public AutoSmartFiveBall(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake, Conveyor conveyor, LimeLight limelight) {
        Pose2d bOrigin = new Pose2d(Units.inchesToMeters(297.657), Units.inchesToMeters(75.108), new Rotation2d(Units.degreesToRadians(90)));
        TrajectoryConfig trajectoryConfig_rev = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint).setReversed(true);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint);    
        first_Pickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(),
            List.of(
                new Translation2d(Units.inchesToMeters(-19.04), 0)
            ), 
            new Pose2d(Units.inchesToMeters(-38.08), 0, new Rotation2d()), trajectoryConfig_rev).relativeTo(bOrigin);
        first_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-38.08), 0, new Rotation2d()),
            List.of(
                new Translation2d(Units.inchesToMeters(-30.69), Units.inchesToMeters(-0.185))
            ), 
            new Pose2d(Units.inchesToMeters(-23.2), Units.inchesToMeters(-0), new Rotation2d(Units.degreesToRadians(-13.215))), trajectoryConfig).relativeTo(bOrigin);
        second_Pickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-23.2), Units.inchesToMeters(-0), new Rotation2d(Units.degreesToRadians(-13.215))),
            List.of(
                new Translation2d(Units.inchesToMeters(-48.089), Units.inchesToMeters(-19.934)),
                new Translation2d(Units.inchesToMeters(-43.115), Units.inchesToMeters(-53.141))
            ), 
            new Pose2d(Units.inchesToMeters(-17.869), Units.inchesToMeters(-81.253), new Rotation2d(Units.degreesToRadians(46.785))), trajectoryConfig_rev).relativeTo(bOrigin);
    
        second_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-17.869), Units.inchesToMeters(-81.253), new Rotation2d(Units.degreesToRadians(46.785))),
            List.of(
                new Translation2d(Units.inchesToMeters(-34.189), Units.inchesToMeters(-51.807))
            ), 
            new Pose2d(Units.inchesToMeters(-2.465), Units.inchesToMeters(-27.138), new Rotation2d(Units.degreesToRadians(-30.626))), trajectoryConfig).relativeTo(bOrigin);
        third_Pickup_Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-2.465), Units.inchesToMeters(-27.138), new Rotation2d(Units.degreesToRadians(-30.626))),
            List.of(
                new Translation2d(Units.inchesToMeters(-48.702), Units.inchesToMeters(-74.498)),
                new Translation2d(Units.inchesToMeters(-19.785), Units.inchesToMeters(-131.68)),
                new Translation2d(Units.inchesToMeters(12.989), Units.inchesToMeters(-182.388))
            ),
            new Pose2d(Units.inchesToMeters(-13.861), Units.inchesToMeters(-238.149), new Rotation2d(Units.degreesToRadians(-47.223))), trajectoryConfig_rev).relativeTo(bOrigin);
        third_Shoot_Trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-13.861), Units.inchesToMeters(-238.149), new Rotation2d(Units.degreesToRadians(-47.223))),
            List.of(
                new Translation2d(Units.inchesToMeters(16.629), Units.inchesToMeters(-205.732))
            ),
            new Pose2d(Units.inchesToMeters(33.09), Units.inchesToMeters(-167.499), new Rotation2d(Units.degreesToRadians(-74.386))), trajectoryConfig).relativeTo(bOrigin);

        Command autoSmartTower1 = new AutoSmartTower(ballTower);
        Command autoSmartTower2 = new AutoSmartTower(ballTower);
        Command autoSmartTower3 = new AutoSmartTower(ballTower);
        Command autoSmartTower4 = new AutoSmartTower(ballTower);
        Command autoSmartShot1 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot2 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot3 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot4 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot5 = new AutoSmartShot(launcher, ballTower);

        RamseteCommand ramseteCommand_first_pickup = new RamseteCommand(
                first_Pickup_trajectory,
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
    
            RamseteCommand ramseteCommand_first_shoot = new RamseteCommand(
                first_Shoot_trajectory,
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
    
            RamseteCommand ramseteCommand_second_pickup = new RamseteCommand(
                second_Pickup_trajectory,
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
    
            RamseteCommand ramseteCommand_second_shoot = new RamseteCommand(
                second_Shoot_trajectory,
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

            RamseteCommand ramseteCommand_third_pickup = new RamseteCommand(
                third_Pickup_Trajectory,
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

            RamseteCommand ramseteCommand_third_shoot = new RamseteCommand(
                third_Shoot_Trajectory,
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
                new InstantCommand(() -> driveTrain.resetOdometry(first_Pickup_trajectory.getInitialPose()), driveTrain),
                new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
                new InstantCommand(conveyor::conveyerForward, conveyor),
                ramseteCommand_first_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new InstantCommand(intake::intakeRollersOff, intake),
                new InstantCommand(intake::intakeUp, intake),
                ramseteCommand_first_shoot.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new InstantCommand(limelight::ledPipeline, limelight),
                new InstantCommand(limelight::startTakingSnapshots, limelight),
                new InstantCommand(limelight::setPipelineOne, limelight),
                new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(.5).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new InstantCommand(limelight::stopTakingSnapshots, limelight),
                new InstantCommand(limelight::setPipelineZero),
                autoSmartShot1.withTimeout(2.25),
                // new RunCommand(() -> ballTower.liftBall(), ballTower).withTimeout(2),
                autoSmartTower1.withTimeout(2),
                autoSmartShot2.withTimeout(2.25),
                new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
                ramseteCommand_second_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                
                new InstantCommand(intake::intakeUp, intake),
                new ParallelCommandGroup(
                    autoSmartTower2.withTimeout(2),
                    ramseteCommand_second_shoot.andThen(() -> driveTrain.tankDriveVolts(0, 0))
                ),
                new InstantCommand(intake::intakeRollersOff, intake),
                new InstantCommand(limelight::setPipelineOne, limelight),
                new InstantCommand(limelight::startTakingSnapshots, limelight),
                new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(.5).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new InstantCommand(limelight::stopTakingSnapshots, limelight),
                new InstantCommand(limelight::setPipelineZero),
                autoSmartShot3.withTimeout(2.25),
                new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
                ramseteCommand_third_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
                new InstantCommand(intake::intakeUp, intake),
                new ParallelCommandGroup(
                    autoSmartTower3.withTimeout(2),
                    ramseteCommand_third_shoot.andThen(() -> driveTrain.tankDriveVolts(0, 0))
                ),
                autoSmartShot4.withTimeout(2.25),
                autoSmartTower4.withTimeout(2),
                autoSmartShot5.withTimeout(2.25),
                new InstantCommand(conveyor::stop, conveyor)
            );
    }
}
