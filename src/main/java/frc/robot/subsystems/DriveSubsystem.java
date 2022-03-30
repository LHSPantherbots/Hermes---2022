package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
// import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.SparkMaxPidConstants;

public class DriveSubsystem extends SubsystemBase {
    //limelight vision may need to be tuned
    private double kP = 0.012;
    private double kF = 0.2;
    

    CANSparkMax leftLeader = new CANSparkMax(3, MotorType.kBrushless);
    CANSparkMax leftFollower = new CANSparkMax(15, MotorType.kBrushless);
    CANSparkMax rightLeader = new CANSparkMax(5, MotorType.kBrushless);
    CANSparkMax rightFollower = new CANSparkMax(6, MotorType.kBrushless);
    
    // Sets up encoders
    RelativeEncoder leftEncoder;
    RelativeEncoder rightEncoder;  

    //Setus up values for closed loop drive
    private SparkMaxPIDController leftPidController;
    private SparkMaxPIDController rightPidController;

    ImuSubsystem m_imuSubsystem;


    DifferentialDrive m_drive = new DifferentialDrive(leftLeader, rightLeader);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;
    private DifferentialDriveOdometry externalOdometry;
    private DifferentialDriveOdometry internalOdometry;

    private DifferentialDrivetrainSim m_simDifDrive;

    private final Field2d m_field = new Field2d();

    public DriveSubsystem(ImuSubsystem imuSubsystem) {
      m_imuSubsystem = imuSubsystem;
      //Resets motor controllers to default conditions
      leftLeader.restoreFactoryDefaults();
      leftFollower.restoreFactoryDefaults();
      rightLeader.restoreFactoryDefaults();
      rightFollower.restoreFactoryDefaults();

      leftLeader.setOpenLoopRampRate(.7);
      rightLeader.setOpenLoopRampRate(.7);

      leftLeader.setSmartCurrentLimit(60);
      leftFollower.setSmartCurrentLimit(60);
      rightLeader.setSmartCurrentLimit(60);
      rightFollower.setSmartCurrentLimit(60);

      rightLeader.setInverted(true);
  
      //Sets up follower motors
      leftFollower.follow(leftLeader);
      rightFollower.follow(rightLeader);

      //Sets up endcoders
      leftEncoder = leftLeader.getEncoder();
      rightEncoder = rightLeader.getEncoder();

      leftPidController = leftLeader.getPIDController();
      rightPidController = rightLeader.getPIDController();

      // set PID coefficients
      
      leftPidController.setP(SparkMaxPidConstants.kPP);
      rightPidController.setP(SparkMaxPidConstants.kPP);
      leftPidController.setI(SparkMaxPidConstants.kI);
      rightPidController.setI(SparkMaxPidConstants.kI);
      leftPidController.setD(SparkMaxPidConstants.kD);
      rightPidController.setD(SparkMaxPidConstants.kD);
      leftPidController.setIZone(SparkMaxPidConstants.kIz);
      rightPidController.setIZone(SparkMaxPidConstants.kIz);
      leftPidController.setFF(SparkMaxPidConstants.kFF);
      rightPidController.setFF(SparkMaxPidConstants.kFF);
      
      leftPidController.setOutputRange(SparkMaxPidConstants.kMinOutput, SparkMaxPidConstants.kMaxOutput);
      rightPidController.setOutputRange(SparkMaxPidConstants.kMinOutput, SparkMaxPidConstants.kMaxOutput);
      
      int smartMotionSlot = 0;
      
      leftPidController.setSmartMotionMaxVelocity(SparkMaxPidConstants.maxVel, smartMotionSlot);
      rightPidController.setSmartMotionMaxVelocity(SparkMaxPidConstants.maxVel, smartMotionSlot);
      leftPidController.setSmartMotionMinOutputVelocity(SparkMaxPidConstants.minVel, smartMotionSlot);
      rightPidController.setSmartMotionMinOutputVelocity(SparkMaxPidConstants.minVel, smartMotionSlot);
      leftPidController.setSmartMotionMaxAccel(SparkMaxPidConstants.maxAcc, smartMotionSlot);
      rightPidController.setSmartMotionMaxAccel(SparkMaxPidConstants.maxAcc, smartMotionSlot);
      
      
      leftPidController.setSmartMotionAllowedClosedLoopError(SparkMaxPidConstants.allowedErr, smartMotionSlot);
      rightPidController.setSmartMotionAllowedClosedLoopError(SparkMaxPidConstants.allowedErr, smartMotionSlot);
      

      resetEncoders();
      
      // Setting the conversion constants on the Encoder objects directly rather than multipling the raw encoder output by the conversion factor manually
      rightEncoder.setPositionConversionFactor(DriveTrainConstants.revsToMeter);
      leftEncoder.setPositionConversionFactor(DriveTrainConstants.revsToMeter);
      rightEncoder.setVelocityConversionFactor(DriveTrainConstants.rpmToMetersPerSec);
      leftEncoder.setVelocityConversionFactor(DriveTrainConstants.rpmToMetersPerSec);

      odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0));

      externalOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
      internalOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

      zeroHeading();

      m_simDifDrive = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(DriveTrainConstants.kvVoltSecondsPerMeter, DriveTrainConstants.kaVoltSecondsSquaredPerMeter, DriveTrainConstants.angular_kvVoltSecondsPerMeter, DriveTrainConstants.angular_kaVoltSecondsSquaredPerMeter),
        DCMotor.getNEO(2), 10.71, DriveTrainConstants.kTrackwidthMeters, Units.inchesToMeters(3), VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

      SmartDashboard.putData("Field", m_field);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run

      // Update the odometry in the periodic block
      var m_pose = odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPositionMeter(),
      getRightEncoderPositionMeter());

      m_drive.feed();

      // Trajectory.State goalState = 
  
  
      //Smart Dashboard Items
      SmartDashboard.putNumber("Left Drive Encoder Position", getLeftEncoderPositionMeter());
      SmartDashboard.putNumber("Right Drive Encoder Position", getRightEncoderPositionMeter());
      SmartDashboard.putNumber("Drive Angle", getHeading());
      //SmartDashboard.putNumber("Right Velocity", getRightEncoderVelocityMeterPerSec());
      //SmartDashboard.putNumber("Left Velocity", getLeftEncoderVelocityMeterPerSec());
      //SmartDashboard.putNumber("Left Drive Applied Voltage", leftLeader.getAppliedOutput());
      //SmartDashboard.putNumber("Right Drive Applied Voltage", rightLeader.getAppliedOutput());
      // SmartDashboard.putNumber("left value", leftLeader.get()*RobotController.getInputVoltage());
      // m_field.setRobotPose(odometry.getPoseMeters());
      m_field.setRobotPose(m_pose);

      //SmartDashboard.updateValues();
    }

    @Override
    public void simulationPeriodic() {
      m_simDifDrive.setInputs(leftLeader.get() * RobotController.getInputVoltage(),
        rightLeader.get() * RobotController.getInputVoltage());
      
      m_simDifDrive.update(0.02);

      leftEncoder.setPosition(m_simDifDrive.getLeftPositionMeters());
      rightEncoder.setPosition(m_simDifDrive.getRightPositionMeters());
      m_imuSubsystem.setAngle(m_simDifDrive.getHeading().getDegrees());
    }

    public void zeroHeading(){
      m_imuSubsystem.resetGyro();
    }


    public void teleopDrive(double move, double turn) {

    if (move < 0)
    {
      move = Math.pow(move, 2) * -1;
    }
    else
    {
      move = Math.pow(move, 2);
    }

    if (turn < 0)
    {
      turn = Math.pow(turn, 2) * -1;
    }
    else
    {
      turn = Math.pow(turn, 2);
    }
    m_drive.arcadeDrive(move, turn);

    }

    public void teleopDrive(double move, double turn, boolean governer) {
      if (governer) {
        m_drive.arcadeDrive(move, turn*.5);
      } else {
        m_drive.arcadeDrive(move, turn);
      }
    }
    
    public void resetEncoders() {
      leftEncoder.setPosition(0);
      rightEncoder.setPosition(0);
    }

    public double getLeftEncoderPosition(){
      return leftEncoder.getPosition();
    }
    
    public double getRightEncoderPosition(){
      return rightEncoder.getPosition();
    }
  
    public double getLeftEncoderPositionMeter(){
      return leftEncoder.getPosition();
    }
  
    public double getRightEncoderPositionMeter(){
      return rightEncoder.getPosition();
    } 
    
    /**
    * Gets the average distance of the two encoders.
    *
    * @return the average of the two encoder readings
    */
    public double getAverageEncoderPositionMeter() {
      return (getLeftEncoderPositionMeter() + getRightEncoderPositionMeter()) / 2.0;
    }
  
    public double getLeftEncoderVelocity(){
      return leftEncoder.getVelocity();
    }
  
    public double getRightEncoderVelocity(){
      return rightEncoder.getVelocity();
    }
  
    public double getLeftEncoderVelocityMeterPerSec(){
      return getLeftEncoderVelocity();
    }
  
    public double getRightEncoderVelocityMeterPerSec(){
      return getRightEncoderVelocity();
    }

    public Pose2d getPose() {
      return odometry.getPoseMeters();
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
      return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocityMeterPerSec(), getRightEncoderVelocityMeterPerSec());
    }
  
    ///Need to add encoder rate meter/second
  
    //Returns heading 180 to -180.  Right turn is negative and Left turn is positive
    public double getHeading(){
      return Math.IEEEremainder(m_imuSubsystem.getAngle(), 360);
    }

    public void resetOdometry(Pose2d pose) {
      resetEncoders();
      odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    
    public void closedLoopDrive(double move, double turn){
      double maxForwardPercent = 1.0;
      double maxTurnTurnPercent = 1.0;
      
  
      //positive velocity makes left go forward
      //negative velocity makes right go forward
  
      double leftVelocity;
      double rightVelocity;
  
      move = move*maxForwardPercent;
      turn = turn*maxTurnTurnPercent; 
  
      double kNorm = Math.abs(move) + Math.abs(turn);
  
      
  
  
      leftVelocity = SparkMaxPidConstants.maxVel * (move + turn)/kNorm;
      rightVelocity = SparkMaxPidConstants.maxVel * (move + turn)/kNorm;
  
      leftPidController.setReference(leftVelocity, CANSparkMax.ControlType.kSmartVelocity);
      rightPidController.setReference(rightVelocity, CANSparkMax.ControlType.kSmartVelocity);
      
    }
    

    /**
    * Controls the left and right sides of the drive directly with voltages.
    *
    * @param leftVolts  the commanded left output
    * @param rightVolts the commanded right output
    */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
      leftLeader.setVoltage(leftVolts);
      rightLeader.setVoltage(rightVolts);
    }
    
    /*
    public void DriveDistance(double distance){
      double error = distance - getLeftEncoderPositionMeter();
      SmartDashboard.putNumber("DriveError", error);
      double dt = Timer.getFPGATimestamp() - lastTimestampDist;
      
      if(Math.abs(error) < iZoneDist){
        errorSumDist = errorSumDist + error * dt;
      }else{
        errorSumDist = 0;
      }
  
      double errorRate = (error - lastErrorDist)/dt;
      double outP = kPD * error;     //Proportional output
      double outI = kID * errorSumDist;  //Intigrator output
      double outD = kDD * errorRate; //Derivitive output
  
      double outputMove = outP + outI + outD;
  
      driveStraight(outputMove);
  
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
    }

    public void TurnAngleManual(double turnAngle){
      double error = getHeading()- turnAngle;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      kFG = Math.copySign(kFG, error);
  
      if(Math.abs(error) < iZone){
        errorSum = errorSum + error * dt;
      }else{
        errorSum = 0;
      }
  
      double errorRate = (error - lastError)/dt;
      double outF = kFG;             //Feed forward output
      double outP = kPG * error;     //Proportional output
      double outI = kIG * errorSum;  //Intigrator output
      double outD = kDG * errorRate; //Derivitive output
  
      double outputTurn = outF + outP + outI + outD;
  
      teleopDrive(0, outputTurn);
  
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
    }
  
    public void TurnAngleAuto(double turnAngle){
      double error = getHeading()- turnAngle;
      double dt = Timer.getFPGATimestamp() - lastTimestamp;
      kFG = Math.copySign(kFG, error);
  
      if(Math.abs(error) < iZone){
        errorSum = errorSum + error * dt;
      }else{
        errorSum = 0;
      }
  
      double errorRate = (error - lastError)/dt;
      double outF = kFG;             //Feed forward output
      double outP = kPA * error;     //Proportional output
      double outI = kIA * errorSum;  //Intigrator output
      double outD = kDA * errorRate; //Derivitive output
  
      double outputTurn = outF + outP + outI + outD;
  
      teleopDrive(0, outputTurn);
  
      lastTimestamp = Timer.getFPGATimestamp();
      lastError = error;
    }
  
    public void driveStraight(double move){
      double error = getHeading();
      double turn = error * gyroKP;
      teleopDrive(move, turn);
      SmartDashboard.putNumber("TurnValue", turn);
  
    }
    */
  
    public boolean isEncoderError() {
      return internalOdometry.getPoseMeters().getTranslation().getDistance(externalOdometry.getPoseMeters().getTranslation()) > 0.5;
    }

    public CANSparkMax getLeftCanSparkMax() {
      return leftLeader;
    }

    public CANSparkMax getRightCanSparkMax() {
      return rightLeader;
    }

    public void limeLightAim()
  {
    double error = RobotContainer.limelight.getHorizontalOffset();
    kF = Math.copySign(kF, error);
    double outF = kF;             
    double outP = kP * error;
    double outputTurn = outF + outP;
    if (RobotContainer.limelight.isTargetValid()){
      teleopDrive(0, outputTurn);
    }else{
      teleopDrive(0, 0);
    }
  }

    
}
