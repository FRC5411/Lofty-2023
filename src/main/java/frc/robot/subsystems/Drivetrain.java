// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Telemetry;
import frc.robot.Constants;
import frc.robot.Constants.ARM.positions;

import static frc.robot.Constants.DRIVETRAIN.*;

import java.util.HashMap;
import java.util.List;

import static frc.robot.Constants.CAN.*;

public class Drivetrain extends SubsystemBase {
  private Pigeon m_gyro;
  //private Arm m_arm;
  //private PinchersofPower m_claw;
  //private Limelight m_limelight;

  private SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    new Translation2d(-ROBOT_WIDTH/2, ROBOT_WIDTH/2),
    new Translation2d(ROBOT_WIDTH/2, ROBOT_WIDTH/2),
    new Translation2d(-ROBOT_WIDTH/2, -ROBOT_WIDTH/2),
    new Translation2d(ROBOT_WIDTH/2, -ROBOT_WIDTH/2)
  );

  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds();
  private ChassisSpeeds forwardKinematics = new ChassisSpeeds();

  private SwerveModuleState[] modules = new SwerveModuleState[3];

  // swerve module CANCoders
  private final CANCoder FL_Position = new CANCoder(FL_CANCODER_ID);
  private final CANCoder FR_Position = new CANCoder(FR_CANCODER_ID);
  private final CANCoder BL_Position = new CANCoder(BL_CANCODER_ID);
  private final CANCoder BR_Position = new CANCoder(BR_CANCODER_ID);

  // swerve module drive motors
  private final TalonFX FL_Drive = new TalonFX(FL_DRIVE_ID);
  private final TalonFX FR_Drive = new TalonFX(FR_DRIVE_ID);
  private final TalonFX BL_Drive = new TalonFX(BL_DRIVE_ID);
  private final TalonFX BR_Drive = new TalonFX(BR_DRIVE_ID);

  // swerve module azimuth (steering) motors
  private final TalonFX FL_Azimuth = new TalonFX(FL_AZIMUTH_ID);
  private final TalonFX FR_Azimuth = new TalonFX(FR_AZIMUTH_ID);
  private final TalonFX BL_Azimuth = new TalonFX(BL_AZIMUTH_ID);
  private final TalonFX BR_Azimuth = new TalonFX(BR_AZIMUTH_ID);

  // swerve module target rotations (degrees)
  private double FL_Target = 0.0;
  private double FR_Target = 0.0;
  private double BL_Target = 0.0;
  private double BR_Target = 0.0;

  // swerve module wheel speeds (percent output)
  private double FL_Speed = 0.0;
  private double FR_Speed = 0.0;
  private double BL_Speed = 0.0;
  private double BR_Speed = 0.0;

  // 'actual' read positions of each swerve module (degrees)
  private double FL_Actual_Position = 0.0;
  private double FR_Actual_Position = 0.0;
  private double BL_Actual_Position = 0.0;
  private double BR_Actual_Position = 0.0;

  // 'actual' read speeds of each swerve module (meters per second)
  private double FL_Actual_Speed = 0.0;
  private double FR_Actual_Speed = 0.0;
  private double BL_Actual_Speed = 0.0;
  private double BR_Actual_Speed = 0.0;

  // robot oriented / field oriented swerve drive toggle
  private boolean isRobotOriented = true;
  
  private static final StatorCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 80, 80, 0);
  private static final StatorCurrentLimitConfiguration AZIMUTH_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 20, 20, 0);

  private SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(0), getSwerveModulePositions(), new Pose2d());

  private Pose2d _robotPose = new Pose2d();

  private double _translationKp = 1;
  private double _translationKi = 0;
  private double _translationKd = 0.75;
  private double _rotationKp = 1;
  private double _rotationKi = 0;
  private double _rotationKd = 1;

  /** Creates a new ExampleSubsystem. */
  public Drivetrain(Pigeon m_gyro/*, Arm m_arm, PinchersofPower m_claw, Limelight m_limelight*/) {
    this.m_gyro = m_gyro;
    /*this.m_arm = m_arm;
    this.m_claw = m_claw;
    this.m_limelight = m_limelight;*/
    
    Telemetry.setValue("drivetrain/PathPlanner/translationKp", _translationKp);
    Telemetry.setValue("drivetrain/PathPlanner/translationKi", _translationKi);
    Telemetry.setValue("drivetrain/PathPlanner/translationKd", _translationKd);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKp", _rotationKp);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKi", _rotationKi);
    Telemetry.setValue("drivetrain/PathPlanner/rotationKd", _rotationKd);

    // config drive motors
    configDrive(FL_Drive);
    configDrive(FR_Drive);
    configDrive(BL_Drive);
    configDrive(BR_Drive);

    // config CANcoders
    configPosition(FL_Position, FL_ECODER_OFFSET);
    configPosition(FR_Position, FR_ECODER_OFFSET);
    configPosition(BL_Position, BL_ECODER_OFFSET);
    configPosition(BR_Position, BR_ECODER_OFFSET);

    // config azimuth (steering) motors
    configAzimuth(FL_Azimuth, FL_Position);
    configAzimuth(FR_Azimuth, FR_Position);
    configAzimuth(BL_Azimuth, BL_Position);
    configAzimuth(BR_Azimuth, BR_Position);

    PathPlannerServer.startServer(6969);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    _translationKp = Telemetry.getValue("drivetrain/PathPlanner/translationKp", 0);
    _translationKi = Telemetry.getValue("drivetrain/PathPlanner/translationKi", 0);
    _translationKd = Telemetry.getValue("drivetrain/PathPlanner/translationKd", 0);
    _rotationKp = Telemetry.getValue("drivetrain/PathPlanner/rotationKp", 0);
    _rotationKi = Telemetry.getValue("drivetrain/PathPlanner/rotationKi", 0);
    _rotationKd = Telemetry.getValue("drivetrain/PathPlanner/rotationKd", 0);

    // 'actual' read sensor positions of each module
    FL_Actual_Position = ((FL_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;
    FR_Actual_Position = ((FR_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;
    BL_Actual_Position = ((BL_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;
    BR_Actual_Position = ((BR_Azimuth.getSelectedSensorPosition() / 4096) * 360) % 360;

    // 'actual' read encoder speeds per module (meters per second)
    FL_Actual_Speed = 2.0*(((FL_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;
    FR_Actual_Speed = 2.0*(((FR_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;
    BL_Actual_Speed = 2.0*(((BL_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;
    BR_Actual_Speed = 2.0*(((BR_Drive.getSelectedSensorVelocity() / 4096) * 10) / DRIVE_GEAR_RATIO) * Math.PI * WHEEL_DIAMETER;

    // dashboard data
    Telemetry.setValue("drivetrain/modules/FL/azimuth/targetPosition", FL_Target);
    Telemetry.setValue("drivetrain/modules/FR/azimuth/targetPosition", FR_Target);
    Telemetry.setValue("drivetrain/modules/BL/azimuth/targetPosition", BL_Target);
    Telemetry.setValue("drivetrain/modules/BR/azimuth/targetPosition", BR_Target);
    Telemetry.setValue("drivetrain/modules/FL/drive/targetSpeed", FL_Speed);
    Telemetry.setValue("drivetrain/modules/FR/drive/targetSpeed", FR_Speed);
    Telemetry.setValue("drivetrain/modules/BL/drive/targetSpeed", BL_Speed);
    Telemetry.setValue("drivetrain/modules/BR/drive/targetSpeed", BR_Speed);
    Telemetry.setValue("drivetrain/modules/FL/azimuth/actualPosition", FL_Actual_Position);
    Telemetry.setValue("drivetrain/modules/FR/azimuth/actualPosition", FR_Actual_Position);
    Telemetry.setValue("drivetrain/modules/BL/azimuth/actualPosition", BL_Actual_Position);
    Telemetry.setValue("drivetrain/modules/BR/azimuth/actualPosition", BR_Actual_Position);
    Telemetry.setValue("drivetrain/modules/FL/drive/actualSpeed", FL_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/FR/drive/actualSpeed", FR_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/BL/drive/actualSpeed", BL_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/BR/drive/actualSpeed", BR_Actual_Speed);
    Telemetry.setValue("drivetrain/modules/FL/drive/temperature", FL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/modules/FR/drive/temperature", FR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/modules/BL/drive/temperature", BL_Drive.getTemperature());
    Telemetry.setValue("drivetrain/modules/BR/drive/temperature", BR_Drive.getTemperature());
    Telemetry.setValue("drivetrain/modules/FL/azimuth/temperature", FL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/modules/FR/azimuth/temperature", FR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/modules/BL/azimuth/temperature", BL_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/modules/BR/azimuth/temperature", BR_Azimuth.getTemperature());
    Telemetry.setValue("drivetrain/modules/FL/azimuth/outputVoltage", FL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FR/azimuth/outputVoltage", FR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BL/azimuth/outputVoltage", BL_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BR/azimuth/outputVoltage", BR_Azimuth.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FL/drive/outputVoltage", FL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FR/drive/outputVoltage", FR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BL/drive/outputVoltage", BL_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/BR/drive/outputVoltage", BR_Drive.getMotorOutputVoltage());
    Telemetry.setValue("drivetrain/modules/FL/azimuth/statorCurrent", FL_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/FR/azimuth/statorCurrent", FR_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BL/azimuth/statorCurrent", BL_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BR/azimuth/statorCurrent", BR_Azimuth.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/FL/drive/statorCurrent", FL_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/FR/drive/statorCurrent", FR_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BL/drive/statorCurrent", BL_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/modules/BR/drive/statorCurrent", BR_Drive.getStatorCurrent());
    Telemetry.setValue("drivetrain/isRobotOriented", isRobotOriented);

    forwardKinematics = m_kinematics.toChassisSpeeds(new SwerveModuleState(FL_Actual_Speed, new Rotation2d(Math.toRadians(FL_Actual_Position))), new SwerveModuleState(FR_Actual_Speed, new Rotation2d(Math.toRadians(FR_Actual_Position))), new SwerveModuleState(BL_Actual_Speed, new Rotation2d(Math.toRadians(BL_Actual_Position))), new SwerveModuleState(BR_Actual_Speed, new Rotation2d(Math.toRadians(BR_Actual_Position))) );

    Telemetry.setValue("drivetrain/kinematics/robot/forwardSpeed", forwardKinematics.vxMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/robot/rightwardSpeed", -forwardKinematics.vyMetersPerSecond);
    Telemetry.setValue("drivetrain/kinematics/clockwiseSpeed", Math.toDegrees(forwardKinematics.omegaRadiansPerSecond));
    Telemetry.setValue("drivetrain/kinematics/field/DSawaySpeed", ( forwardKinematics.vxMetersPerSecond * Math.cos(Math.toRadians(m_gyro.getYaw())) - forwardKinematics.vyMetersPerSecond * Math.sin(Math.toRadians(m_gyro.getYaw()))));
    Telemetry.setValue("drivetrain/kinematics/field/DSrightSpeed", ( -forwardKinematics.vyMetersPerSecond * Math.cos(Math.toRadians(m_gyro.getYaw())) - forwardKinematics.vxMetersPerSecond * Math.sin(Math.toRadians(m_gyro.getYaw()))));

    //_robotPose = m_odometry.update(new Rotation2d(Math.toRadians(m_gyro.getYaw())), new SwerveModuleState(FL_Actual_Speed, new Rotation2d(Math.toRadians(FL_Actual_Position))), new SwerveModuleState(FR_Actual_Speed, new Rotation2d(Math.toRadians(FR_Actual_Position))), new SwerveModuleState(BL_Actual_Speed, new Rotation2d(Math.toRadians(BL_Actual_Position))), new SwerveModuleState(BR_Actual_Speed, new Rotation2d(Math.toRadians(BR_Actual_Position))) );
    //m_odometry.addVisionMeasurement(m_limelight.getPose(), Timer.getFPGATimestamp() - m_limelight.getLatency());
    _robotPose = m_odometry.update(new Rotation2d(m_gyro.getYaw()), getSwerveModulePositions());

    Telemetry.setValue("drivetrain/odometry/field/DSawayPosition", -_robotPose.getX());
    Telemetry.setValue("drivetrain/odometry/field/DSrightPosition", _robotPose.getY());

    Telemetry.setValue("general/FMSAlliance", DriverStation.getAlliance() == Alliance.Blue ? "Blue" : (DriverStation.getAlliance() == Alliance.Red ? "Red" : "Invalid") );
    Telemetry.setValue("general/joystick0Name", DriverStation.getJoystickName(0));
    Telemetry.setValue("general/joystick1Name", DriverStation.getJoystickName(1));
  }

  @Override
  public void simulationPeriodic() {}

  public void joystickDrive(double LX, double LY, double RX) {

    // WPILib swerve command
    m_chassisSpeeds = new ChassisSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED);
    if ( !isRobotOriented ) m_chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(LY * MAX_LINEAR_SPEED, -LX * MAX_LINEAR_SPEED, -RX * MAX_ROTATION_SPEED, Rotation2d.fromDegrees(m_gyro.getYaw()));
    
    modules = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);

    if ( LX == 0 && LY == 0 && RX == 0 ) {
      FL_Azimuth.set(ControlMode.PercentOutput, 0);
      FR_Azimuth.set(ControlMode.PercentOutput, 0);
      BL_Azimuth.set(ControlMode.PercentOutput, 0);
      BR_Azimuth.set(ControlMode.PercentOutput, 0);
      FL_Drive.set(ControlMode.PercentOutput, 0);
      FR_Drive.set(ControlMode.PercentOutput, 0);
      BL_Drive.set(ControlMode.PercentOutput, 0);
      BR_Drive.set(ControlMode.PercentOutput, 0);
      return;
    }

    driveFromModuleStates(modules);
  }

  public void driveFromModuleStates ( SwerveModuleState[] modules ) {
    SwerveDriveKinematics.desaturateWheelSpeeds(modules, MAX_LINEAR_SPEED);

    FL_Target = modules[0].angle.getDegrees() % 360;
    FR_Target = modules[1].angle.getDegrees() % 360;
    BL_Target = modules[2].angle.getDegrees() % 360;
    BR_Target = modules[3].angle.getDegrees() % 360;
    FL_Speed = modules[0].speedMetersPerSecond;
    FR_Speed = modules[1].speedMetersPerSecond;
    BL_Speed = modules[2].speedMetersPerSecond;
    BR_Speed = modules[3].speedMetersPerSecond;

    // find the shortest path to an equivalent position to prevent unneccesary full rotations
    FL_Target = optimizeAzimuthPath(FL_Target, FL_Actual_Position);
    FR_Target = optimizeAzimuthPath(FR_Target, FR_Actual_Position);
    BL_Target = optimizeAzimuthPath(BL_Target, BL_Actual_Position);
    BR_Target = optimizeAzimuthPath(BR_Target, BR_Actual_Position);

    // correct the target positions so that they are close to the current position
    // then convert to sensor units and pass target positions to motor controllers
    FL_Azimuth.set(ControlMode.Position, ((FL_Target + (FL_Actual_Position - (FL_Actual_Position % 360))) / 360) * 4096);
    FR_Azimuth.set(ControlMode.Position, ((FR_Target + (FR_Actual_Position - (FR_Actual_Position % 360))) / 360) * 4096);
    BL_Azimuth.set(ControlMode.Position, ((BL_Target + (BL_Actual_Position - (BL_Actual_Position % 360))) / 360) * 4096);
    BR_Azimuth.set(ControlMode.Position, ((BR_Target + (BR_Actual_Position - (BR_Actual_Position % 360))) / 360) * 4096);

    // pass wheel speeds to motor controllers
    FL_Drive.set(ControlMode.Velocity, (FL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*4096)/10);
    FR_Drive.set(ControlMode.Velocity, (FR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*4096)/10);
    BL_Drive.set(ControlMode.Velocity, (BL_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*4096)/10);
    BR_Drive.set(ControlMode.Velocity, (BR_Speed*DRIVE_GEAR_RATIO/(Math.PI * WHEEL_DIAMETER)*4096)/10);
  }

  public Command moveToPositionCommand (Pose2d target) { // TODO move to position command
    PathPlannerTrajectory _toTarget = PathPlanner.generatePath(
      PathPlanner.getConstraintsFromPath(Telemetry.getValue("general/autonomous/selectedRoutine", "default")),
      new PathPoint(new Translation2d(m_odometry.getEstimatedPosition().getX(),m_odometry.getEstimatedPosition().getY()), new Rotation2d(m_gyro.getYaw()), Math.hypot(forwardKinematics.vxMetersPerSecond, forwardKinematics.vxMetersPerSecond)),
      new PathPoint(new Translation2d(target.getX(), target.getY()), target.getRotation())
    );
    return new PPSwerveControllerCommand(
      _toTarget,
      () -> m_odometry.getEstimatedPosition(), // Pose2d supplier
      this.m_kinematics, // SwerveDriveKinematics
      new PIDController(_translationKp, _translationKi, _translationKd), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDController(_rotationKp, _rotationKi, _rotationKd), // PID constants to correct for rotation error (used to create the rotation controller)
      new PIDController(_rotationKp, _rotationKi, _rotationKd), // PID constants to correct for rotation error (used to create the rotation controller)
      this::driveFromModuleStates, // Module states consumer used to output to the drive subsystem
      (Subsystem) this
    );
  }

  /** Sets the gyroscope's current heading to 0 */
  public void zeroGyro() {
    m_gyro.zeroYaw();
  }

  /** toggles field/robot orientation
   * @return new isRobotOriented value
   */
  public boolean toggleRobotOrient() {
    isRobotOriented = !isRobotOriented;
    return isRobotOriented;
  }

  /** @return true if robot oriented, false if field oriented */
  public boolean getIsRobotOriented() {
    return isRobotOriented;
  }

  /** Sets the robot's orientation to robot (true) or field (false) 
   * @param _isRobotOriented - false if the robot should move with the gyro
  */
  public void setRobotOriented(boolean _isRobotOriented) {
    isRobotOriented = _isRobotOriented;
  }

  /** runs the configuration methods to apply the config variables 
   * @param motor - the device to configure
  */
  private void configDrive (TalonFX motor) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configStatorCurrentLimit(DRIVE_CURRENT_LIMIT);
    motor.config_kP(0, DRIVE_kP);
    motor.config_kF(0, DRIVE_kF);
  }

  /** runs the configuration methods to apply the config variables 
   * @param motor - the device to configure
   * @param position - the CANCoder associated with the module
  */
  private void configAzimuth (TalonFX motor, CANCoder position) {
    motor.configFactoryDefault();
    motor.setInverted(TalonFXInvertType.CounterClockwise);
    motor.setNeutralMode(NeutralMode.Brake);
    motor.configRemoteFeedbackFilter(position, 0);
    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0);
    motor.configStatorCurrentLimit(AZIMUTH_CURRENT_LIMIT);
    motor.setSelectedSensorPosition(FL_Position.getAbsolutePosition());
    motor.config_kP(0, AZIMUTH_kP);
    motor.config_kD(0, AZIMUTH_kD);
  }
  
  /** runs the configuration methods to apply the config variables 
   * @param encoder - the device to configure
   * @param offset - the measured constant offset in degrees
  */
  private void configPosition (CANCoder encoder, double offset) {
    encoder.configFactoryDefault();
    encoder.configMagnetOffset(offset);
    encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    encoder.setPositionToAbsolute();
  }

  /** finds the closest equivalent position to the target 
   * @param target - the target direction of the module (deg)
   * @param actual - the current direction of the module (deg)
   * @return the corrected target position of the motor
  */
  private double optimizeAzimuthPath (double target, double actual) {
    if (Math.min(Math.min(Math.abs(target - actual), Math.abs((target + 360) - actual)), Math.abs((target - 360) - actual)) == Math.abs((target + 360) - actual))
      target += 360;
    if (Math.min(Math.min(Math.abs(target - actual), Math.abs((target + 360) - actual)), Math.abs((target - 360) - actual)) == Math.abs((target - 360) - actual))
      target -= 360;
    return target;
  }

  private SwerveModulePosition[] getSwerveModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = new SwerveModulePosition((FL_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO, new Rotation2d(FL_Actual_Position));
    positions[1] = new SwerveModulePosition((FR_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO, new Rotation2d(FR_Actual_Position));
    positions[2] = new SwerveModulePosition((BL_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO, new Rotation2d(BL_Actual_Position));
    positions[3] = new SwerveModulePosition((BR_Drive.getSelectedSensorPosition() / 2048) * Constants.DRIVETRAIN.DRIVE_GEAR_RATIO, new Rotation2d(BR_Actual_Position));
    return positions;
  }

  public Command getAutonomousCommand () {
    return new   InstantCommand();/*
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
    // for every path in the group
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
      Telemetry.getValue("general/autonomous/selectedRoutine", "default"), 
      PathPlanner.getConstraintsFromPath(Telemetry.getValue("general/autonomous/selectedRoutine", "default"))
    );

    // This is just an example event map. It would be better to have a constant, global event map
    // in your code that will be used by all path following commands.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("Passed marker 1"));
    eventMap.put("placeHigh", m_arm.moveToPositionCommand(positions.ScoreHigh));
    eventMap.put("release", m_claw.outtakeCommand());
    eventMap.put("pickupLow", m_arm.moveToPositionCommand(positions.ScoreLow));
    eventMap.put("intakeIn", new FunctionalCommand(null, () -> {
      m_claw.intakeCommand();
    }, 
    null, 
    () -> {
      return false; // TODO color sensor
    }, 
    m_claw));
    eventMap.put("autobalance", new InstantCommand()); // TODO balance command
    eventMap.put("realign", moveToPositionCommand(new Pose2d())); // TODO align command
    eventMap.put("coneMode", new InstantCommand( () -> { m_claw.setMode("cone"); } ));
    eventMap.put("cubeMode", new InstantCommand( () -> { m_claw.setMode("cube"); } ));

    // Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      () -> m_odometry.getEstimatedPosition(), // Pose2d supplier
      pose -> m_odometry.resetPosition(new Rotation2d(Math.toRadians(m_gyro.getYaw())), getSwerveModulePositions(), pose), // Pose2d consumer, used to reset odometry at the beginning of auto
      this.m_kinematics, // SwerveDriveKinematics
      new PIDConstants(_translationKp, _translationKi, _translationKd), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(_rotationKp, _rotationKi, _rotationKd), // PID constants to correct for rotation error (used to create the rotation controller)
      this::driveFromModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
      (Subsystem) this // The drive subsystem. Used to properly set the requirements of path following commands
    );

    return autoBuilder.fullAuto(pathGroup);*/
  }
}

// TODO targeting
// TODO shwerve