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


//Untested

public class ThreeBallAuto  extends SequentialCommandGroup {
    
    Trajectory first_Pickup_trajectory = new Trajectory();
    Trajectory first_Shoot_trajectory = new Trajectory();
    Trajectory second_Pickup_trajectory = new Trajectory();
    Trajectory second_Shoot_trajectory = new Trajectory();
    RamseteCommand ramseteCommand_first_pickup;
    RamseteCommand ramseteCommand_first_shoot;
    RamseteCommand ramseteCommand_second_pickup;
    RamseteCommand ramseteCommand_second_shoot; 
 

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
    
    
    public ThreeBallAuto(DriveSubsystem driveTrain, Launcher launcher, BallTower ballTower, Intake intake, Conveyor conveyor) {
        


    TrajectoryConfig trajectoryConfig_rev = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint).setReversed(true);
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(DriveTrainConstants.kMaxSpeedMetersPerSecond,
        DriveTrainConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveTrainConstants.kDriveKinematics).addConstraint(autoVoltageConstraint).addConstraint(centripetalAccelerationConstraint).addConstraint(ddKinematicConstraint).addConstraint(maxVelocityConstraint);    
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
        new Pose2d(), trajectoryConfig);

    second_Pickup_trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0,0, new Rotation2d()),
            List.of(
                new Translation2d(-0.43, 0.93)
            ), 
            new Pose2d(-0.5, 2.75, new Rotation2d(-0.7853)), trajectoryConfig_rev);
    
    second_Shoot_trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(-0.5, 2.75, new Rotation2d(-0.7853)),
                List.of(
                    new Translation2d(0.2, 2.0)
                ), 
                new Pose2d(0.6, 1.6, new Rotation2d(-0.7853)), trajectoryConfig);

    Command waitForLauncher1 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher2 = new WaitForLauncherAtSpeed(launcher);
    Command waitForLauncher3 = new WaitForLauncherAtSpeed(launcher);
    Command waitForBeamBreak = new WaitForBeamBreak(ballTower);

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
        new InstantCommand(() -> driveTrain.resetOdometry(first_Pickup_trajectory.getInitialPose()), driveTrain),
        new InstantCommand(() -> intake.intakeDownnRoll(), intake),
        new InstantCommand(() -> conveyor.conveyerForward(), conveyor),
        ramseteCommand_first_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> intake.intakeRollersOff(), intake),
        new InstantCommand(() -> intake.intakeUp(), intake),
        ramseteCommand_first_shoot.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> launcher.midTarmacShoot(), launcher),
        waitForLauncher1,
        new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower),
        new InstantCommand(() -> ballTower.liftBall(), ballTower),
        waitForBeamBreak,
        waitForLauncher2,
        new RunCommand(() -> ballTower.feedBallToLauncher(), ballTower).withTimeout(2),

        new InstantCommand(() -> intake.intakeDownnRoll(), intake),
        ramseteCommand_second_pickup.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> intake.intakeRollersOff(), intake),
        new InstantCommand(() -> intake.intakeUp(), intake),
        new InstantCommand(() -> ballTower.liftBall(), ballTower),
        ramseteCommand_second_shoot.andThen(() -> driveTrain.tankDriveVolts(0, 0)),
        new InstantCommand(() -> ballTower.feedBallToLauncher(), ballTower).withTimeout(1),
        new InstantCommand(() -> conveyor.stop(), conveyor),
        new InstantCommand(() -> launcher.setVelocitySetpoint(0), launcher),
        waitForLauncher3
        );
    }
}