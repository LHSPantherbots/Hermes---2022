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

public class AutoSmartThreeBall  extends SequentialCommandGroup {
    Trajectory first_Pickup_trajectory = new Trajectory();
    Trajectory first_Shoot_trajectory = new Trajectory();
    Trajectory second_Pickup_trajectory = new Trajectory();
    Trajectory second_Shoot_trajectory = new Trajectory();
    RamseteCommand ramseteCommand_first_pickup;
    RamseteCommand ramseteCommand_first_shoot;
    RamseteCommand ramseteCommand_second_pickup;
    RamseteCommand ramseteCommand_second_shoot;
    Command autoSmartShot1; 
    Command autoSmartShot2; 
    Command autoSmartShot3; 
    Command autoSmartTower1;
    Command autoSmartTower2;
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
            new Pose2d(),
            List.of(
                new Translation2d(-0.75, 0)
            ), 
            new Pose2d(-1.5, 0, new Rotation2d()), trajectoryConfig_rev);
        first_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-1.5, 0, new Rotation2d()),
            List.of(
                new Translation2d(-0.75, 0)
            ), 
            new Pose2d(-0.05, 0, new Rotation2d()), trajectoryConfig);

        second_Pickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.05, 0, new Rotation2d()),
            List.of(
                new Translation2d(-0.66, 1.32)
            ), 
            new Pose2d(-0.83, 2.75, new Rotation2d(-0.15)), trajectoryConfig_rev);
    
        second_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(-0.83, 2.75, new Rotation2d(-0.15)),
            List.of(
                new Translation2d(-0.43, 2.4)
            ), 
            new Pose2d(0.4, 0.91, new Rotation2d(-0.54)), trajectoryConfig);
        
        Command autoSmartTower1 = new AutoSmartTower(ballTower);
        Command autoSmartTower2 = new AutoSmartTower(ballTower);
        Command autoSmartShot1 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot2 = new AutoSmartShot(launcher, ballTower);
        Command autoSmartShot3 = new AutoSmartShot(launcher, ballTower);

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
            new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(2).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new InstantCommand(limelight::stopTakingSnapshots, limelight),
            new InstantCommand(limelight::setPipelineZero),
            autoSmartShot1.withTimeout(.5),
            autoSmartTower1.withTimeout(2),
            autoSmartShot2.withTimeout(.5),
            new InstantCommand(() -> intake.intakeDownnRoll(), intake).alongWith(new InstantCommand(ballTower::runTowerRoller, ballTower)),
            ramseteCommand_second_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new InstantCommand(intake::intakeRollersOff, intake),
            new InstantCommand(intake::intakeUp, intake),
            new ParallelCommandGroup(
                autoSmartTower2.withTimeout(2),
                ramseteCommand_second_shoot.andThen(() -> driveTrain.tankDriveVolts(0, 0))
            ),
            new InstantCommand(limelight::setPipelineOne, limelight),
            new InstantCommand(limelight::startTakingSnapshots, limelight),
            new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(2).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
            new InstantCommand(limelight::stopTakingSnapshots, limelight),
            new InstantCommand(limelight::setPipelineZero),
            autoSmartShot3.withTimeout(.5),
            new InstantCommand(conveyor::stop, conveyor)
        );
    }
    
}
