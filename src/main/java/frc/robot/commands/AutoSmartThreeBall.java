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

public class AutoSmartThreeBall  extends SequentialCommandGroup {
    Trajectory first_Pickup_trajectory = new Trajectory();
    Trajectory first_Shoot_trajectory = new Trajectory();
    Trajectory second_Pickup_trajectory = new Trajectory();
    Trajectory second_Shoot_trajectory = new Trajectory();
    Trajectory intakeDown_trajectory = new Trajectory();
    RamseteCommand ramseteCommand_first_pickup;
    RamseteCommand ramseteCommand_first_shoot;
    RamseteCommand ramseteCommand_second_pickup;
    RamseteCommand ramseteCommand_second_shoot;
    Command autoSmartShot1; 
    Command autoSmartShot2; 
    Command autoSmartShot3;
    Command autoSmartShot4;
    Command autoSmartTower1;
    Command autoSmartTower2;
    Command autoSmartTower3;
    private final PIDController left_PidController = new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    private final PIDController right_PidController =new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    public DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
        DriveTrainConstants.kDriveKinematics, 10);

    public DifferentialDriveKinematicsConstraint ddKinematicConstraint = new DifferentialDriveKinematicsConstraint(DriveTrainConstants.kDriveKinematics, DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public AutoSmartThreeBall(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake, Conveyor conveyor, LimeLight limelight) {
        TrajectoryConfig trajectoryConfig_rev = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint).setReversed(true);
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(DriveTrainConstants.centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint);    
        first_Pickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
            List.of(
                new Translation2d(Units.inchesToMeters(-12.75), 0)
            ), 
            new Pose2d(Units.inchesToMeters(-43), 0, new Rotation2d()), trajectoryConfig_rev);
        first_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-43), 0, new Rotation2d()),
            List.of(
                new Translation2d(Units.inchesToMeters(-30.25), Units.inchesToMeters(-1.113))
            ), 
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(-3), new Rotation2d(Units.degreesToRadians(-8))), trajectoryConfig);
            intakeDown_trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(Units.inchesToMeters(0), 0, new Rotation2d()),
                List.of(
                    new Translation2d(Units.inchesToMeters(2.5), Units.inchesToMeters(0))
                ), 
                new Pose2d(Units.inchesToMeters(5), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))), trajectoryConfig);
        second_Pickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-10), Units.inchesToMeters(-3), new Rotation2d(Units.degreesToRadians(-8))),
            List.of(
                new Translation2d(Units.inchesToMeters(-35), Units.inchesToMeters(28.5))
            ), 
            new Pose2d(Units.inchesToMeters(-5), Units.inchesToMeters(85), new Rotation2d(Units.degreesToRadians(-110))), trajectoryConfig_rev);
    
        second_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(-5), Units.inchesToMeters(85), new Rotation2d(Units.degreesToRadians(-110))),
            List.of(
                new Translation2d(Units.inchesToMeters(-8), Units.inchesToMeters(72))
            ), 
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(60), new Rotation2d(Units.degreesToRadians(-35))), trajectoryConfig);
        
        Command autoSmartTower1 = new AutoSmartTower(ballTower);
        Command autoSmartTower2 = new AutoSmartTower(ballTower);
        Command autoSmartTower3 = new AutoSmartTower(ballTower);
        Command autoSmartShot1 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot2 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot3 = new AutoSmartShotPurple(launcher, ballTower);
        Command autoSmartShot4 = new AutoSmartShotPurple(launcher, ballTower);

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

        RamseteCommand ramseteCommand_intake_Down_0 = new RamseteCommand(
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

        addCommands(
            new InstantCommand(() -> driveTrain.resetEncoders(), driveTrain),
            new InstantCommand(() -> driveTrain.zeroHeading(), driveTrain),
            new InstantCommand(() -> driveTrain.resetOdometry(first_Pickup_trajectory.getInitialPose()), driveTrain),
            new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
            ramseteCommand_intake_Down_0.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
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
            autoSmartShot1.withTimeout(1.5),
            // new RunCommand(() -> ballTower.liftBall(), ballTower).withTimeout(2),
            autoSmartTower1.withTimeout(1.5),
            autoSmartShot2.withTimeout(1.5),
            new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
            ramseteCommand_second_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new InstantCommand(intake::intakeRollersOff, intake),
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
            autoSmartTower3.withTimeout(2),
            autoSmartShot4.withTimeout(2.25),
            new InstantCommand(conveyor::stop, conveyor)
        );
    }
    
}
