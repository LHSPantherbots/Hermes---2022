// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class CAN_ID{
        //Drivetrain
        public static final int DRIVE_LEFT_LEADER = 3;
        public static final int DRIVE_LEFT_FOLLOWER = 15;
        public static final int DRIVE_RIGHT_LEADER = 5;
        public static final int DRIVE_RIGHT_FOLLOWER = 6;
        
        //Intake
        public static final int INTAKE_ROLLER = 4;
        public static final int COVEYOR = 8;

        //Ball Tower
        public static final int BALL_TOWER_ROLLER = 7;
        public static final int BALL_TOWER_BELTS = 12;

        //Ball Ejector
        public static final int BALL_EJECTOR = 11;

        //Climber
        public static final int CLIMB_LEFT = 13;
        public static final int CLIMB_RIGHT = 14;


        // Launcher
        public static final int LAUNCH_LEADER = 9;

        public static final int LAUNCH_FOLLOWER = 10;

        public static final int LAUNCH_TOPROLLER = 17;
    
    
    }

    public static final class RIO_Channels_DIO{
        public static final int TOWER_BEAM_BREAK = 0;
        public static final int EJECTOR_BEAM_BREAK = 1;
        public static final int COLOR_SENSOR_RED = 8;
        public static final int COLOR_SENSOR_BLUE = 9;
    }

    public static final class PH_Channel{
        public static final int INTAKE_A = 3;
        public static final int INTAKE_B = 4;
        public static final int HOOD = 0;
        public static final int CLIMB_A = 1;
        public static final int CLIMB_B = 2;
    }
    public static final class GamePadButtons{
        // Basic buttons
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int RB = 6;
        public static final int LB = 5;
        public static final int Select = 7;
        public static final int Start = 8;
        public static final int LeftJ = 9;
        public static final int RightJ = 10;

        //POV buttons
        public static final int Up = 0;  
        public static final int Down = 180;   
        public static final int Left = 270;
        public static final int Right = 90; 

        //Axis
        public static final int leftY = 1;
        public static final int leftX = 0;
        public static final int rightY = 5;
        public static final int rightX = 4;
        public static final int leftTrigger = 2;
        public static final int rightTrigger = 3;
    }

    public static final class RuleLimits {
        public static final int maxBalls = 2;
    }

    public static final class DriveTrainConstants {
        public static final double ksVolts = 0.21074;
        public static final double kvVoltSecondsPerMeter = 2.8111;
        public static final double kaVoltSecondsSquaredPerMeter = 0.58838;
        public static final double angular_ksVolts = 0.16242;
        public static final double angular_kvVoltSecondsPerMeter = 3.1275;
        public static final double angular_kaVoltSecondsSquaredPerMeter = 0.10085;
        public static final double kPDriveVel = 3.899;
        public static final double left_Ks = 0.14208;
        public static final double left_Kv = 3.2143;
        public static final double left_Ka = 0.16964;
        public static final double left_Kp = 3.0601;
        public static final double right_Ks = 0.10944;
        public static final double right_Kv = 3.4427;
        public static final double right_Ka = 0.24607;
        public static final double right_Kp = 3.4966;
        // public static final double kTrackwidthMeters = Units.inchesToMeters(24);
        public static final double kTrackwidthMeters = 0.62675;
        public static final double kMaxSpeedMetersPerSecond = 1.75;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.25;
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
        public static final double gearRatio = 10.71;
        public static final double wheelDiameter = Units.inchesToMeters(6);
        // Encoder Ticks to Meter Conversion factor (Calculates the Circumferance of a
        // 6" wheel converts to m and then divides by the number of ticks per rev)
        public static final double ticksToMeter = (Math.PI * wheelDiameter * 0.0254)/10.75;
        public static final double revsToMeter = (wheelDiameter*Math.PI)/gearRatio;
        // Encoder Velocity RPM to Wheel Surface Speed meter/second conversion factor
        public static final double rpmToMeterPerSec = (Math.PI * 6 * 0.0254)/10.75 / 60;
        public static final double rpmToMetersPerSec = revsToMeter/60;
        public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);
        public static final CentripetalAccelerationConstraint centripetalAccelerationConstraint = new CentripetalAccelerationConstraint(1);
    }
    public static final class ArmPidConstants {
        public static final double kP = 0.0000035;
        public static final double kI = 0.0;
        public static final double kD = 0.00002; 
        public static final double kIz = 0;
        public static final double kFF = 0.000165;
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final double maxRPM = 5700;
        public static final double maxVel = 5700;
        public static final double minVel = -5700;
        public static final double maxAcc = 4000;
        public static final double allowedErr = 0;
    }

    public static final class SparkMaxPidConstants {
        // PID Coefficients
        public static final double kPP = 5e-5;
        public static final double kI = 1e-6;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0.000156; 
        public static final double kMaxOutput = 1;
        public static final double kMinOutput = -1;
        public static final double maxRPM = 5700;
        // Smart Motion Coefficients
        public static final double maxVel = 5700; // Same as RPM
        public static final double maxAcc = 4000;
        public static final double minVel = 0;
        public static final double allowedErr = 0;
    }

    public static final class LimeL{
        //Leds
        public static final int ledOFF = 1;
        public static final int ledON = 3;

        //pipelines

    }

}
