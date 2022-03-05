package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
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
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class AutoCommand  extends SequentialCommandGroup {
    String redPickup_trajectoryJSON = "paths/Red_Pickup.wpilib.json";
    Trajectory redPickup_trajectory = new Trajectory();
    String redShoot_trajectoryJSON = "paths/Red_Shoot.wpilib.json";
    Trajectory redShoot_trajectory = new Trajectory();
    RamseteCommand ramseteCommand1;
    RamseteCommand ramseteCommand2; 
    Command waitForLauncher1; 
    Command waitForLauncher2; 
    Command waitForLauncher3;
    Command waitForBeamBreak;
    private final PIDController left_PidController = new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    private final PIDController right_PidController =new PIDController(DriveTrainConstants.kPDriveVel, 0, 0);
    public DifferentialDriveVoltageConstraint autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
        new SimpleMotorFeedforward(DriveTrainConstants.ksVolts, DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter), 
        DriveTrainConstants.kDriveKinematics, 10);
    public CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(3);

    public DifferentialDriveKinematicsConstraint ddKinematicConstraint = new DifferentialDriveKinematicsConstraint(DriveTrainConstants.kDriveKinematics, DriveTrainConstants.kMaxSpeedMetersPerSecond);

    public MaxVelocityConstraint maxVelocityConstraint = new MaxVelocityConstraint(DriveTrainConstants.kMaxSpeedMetersPerSecond);
    public AutoCommand(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake) {
        
    // try {
    //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(redPickup_trajectoryJSON);
    //     redPickup_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        
    // } catch (IOException ex) {
    //     DriverStation.reportError("Unable to open trajectory: " + redPickup_trajectoryJSON, ex.getStackTrace());
    // }

    // try {
    //     Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(redShoot_trajectoryJSON);
    //     redShoot_trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    // } catch (IOException ex) {
    //     DriverStation.reportError("Unable to open trajectory: " + redShoot_trajectoryJSON, ex.getStackTrace());
    // }

    TrajectoryConfig trajectoryConfig_rev = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint).setReversed(true);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint);    
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
        new Pose2d(), trajectoryConfig);
    
    Command waitForLauncher1 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher2 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher3 = new WaitForLauncherAtSpeed(launcher);
    Command waitForBeamBreak = new WaitForBeamBreak(ballTower);

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
        new InstantCommand(() -> driveTrain.resetOdometry(redPickup_trajectory.getInitialPose()), driveTrain),
        new InstantCommand(() -> intake.intakeDownnRoll(), intake),
        ramseteCommand1.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> intake.intakeRollersOff(), intake),
        new InstantCommand(() -> intake.intakeUp(), intake),
        ramseteCommand2.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> launcher.midTarmacShoot(), launcher),
        waitForLauncher1,
        new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        new InstantCommand(() -> ballTower.liftBall(), ballTower),
        waitForBeamBreak.raceWith(new InstantCommand(() -> driveTrain. tankDriveVolts(0,0), driveTrain).withTimeout(4)),
        //waitForBeamBreak,
        waitForLauncher2,
        new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        new InstantCommand(() -> launcher.setVelocitySetpoint(0), launcher),
        waitForLauncher3
        );
    }
}
