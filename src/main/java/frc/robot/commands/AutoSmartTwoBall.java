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

public class AutoSmartTwoBall extends SequentialCommandGroup {
    Trajectory redPickup_trajectory = new Trajectory();
    Trajectory redShoot_trajectory = new Trajectory();
    RamseteCommand ramseteCommand1;
    RamseteCommand ramseteCommand2; 
    Command autoSmartShot1; 
    Command autoSmartShot2; 
    Command autoSmartTower;
    private final PIDController left_PidController = new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    private final PIDController right_PidController =new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    public DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
        DriveTrainConstants.kDriveKinematics, 10);

    public DifferentialDriveKinematicsConstraint ddKinematicConstraint = new DifferentialDriveKinematicsConstraint(DriveTrainConstants.kDriveKinematics, DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public AutoSmartTwoBall(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake, Conveyor conveyor, LimeLight limelight) {
        
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
    
    Command autoSmartTower = new AutoSmartTower(ballTower);
    Command autoSmartShot1 = new AutoSmartShot(launcher, ballTower);
    Command autoSmartShot2 = new AutoSmartShot(launcher, ballTower);

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
        // new InstantCommand(() -> intake.intakeRollersOff(), intake),
        new InstantCommand(intake::intakeRollersOff, intake),
        // new InstantCommand(() -> intake.intakeUp(), intake),
        new InstantCommand(intake::intakeUp, intake),
        ramseteCommand2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(limelight::ledPipeline, limelight),
        // new RunCommand(() -> limelight.ledPipeline(), limelight).withTimeout(.1),
        // new RunCommand(() -> limelight.setPipeline(1), limelight).withTimeout(.1),
        new InstantCommand(limelight::setPipelineOne, limelight),
        new RunCommand(() -> driveTrain.limeLightAim(), driveTrain).withTimeout(2).andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        // new RunCommand(() -> limelight.setPipeline(0), limelight).withTimeout(.1),
        new InstantCommand(limelight::setPipelineZero),
        autoSmartShot1.withTimeout(.5),
        autoSmartTower.withTimeout(2),
        autoSmartShot2.withTimeout(.5),
        new InstantCommand(conveyor::stop, conveyor)
        );
    }
}
