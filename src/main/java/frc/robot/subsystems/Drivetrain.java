// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//old
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// new
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import frc.robot.Constants;
import frc.robot.Constants.DrivetrainConstants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;


public class Drivetrain extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  
  //private final Spark m_leftMotor = new Spark(0);
  //private final Spark m_rightMotor = new Spark(1);

  private final WPI_TalonFX m_leftLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftBackTalonFX);
  private final WPI_TalonFX m_rightLeader = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightBackTalonFX);
  private final WPI_TalonFX m_leftFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainLeftFrontTalonFX);
  private final WPI_TalonFX m_rightFollower = new WPI_TalonFX(Constants.CANBusIDs.kDrivetrainRightFrontTalonFX);

  private Supplier<Transmission.GearState> m_gearStateSupplier;

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  //old 
  // private final Encoder m_leftEncoder = new Encoder(4, 5);
 // private final Encoder m_rightEncoder = new Encoder(6, 7);
// encoders part of motors 
  // Set up the differential drive controller

  // private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // Set up the RomiGyro
  // private final RomiGyro m_gyro = new RomiGyro();
  private DifferentialDrive m_diffDrive;

  // Set up the BuiltInAccelerometer
  // private final BuiltInAccelerometer m_accelerometer = new BuiltInAccelerometer();
  private WPI_PigeonIMU m_pigeon = new WPI_PigeonIMU(Constants.CANBusIDs.kPigeonIMU);

  /** Creates a new Drivetrain. */
  // OLD public Drivetrain() {

  public Drivetrain(Supplier<Transmission.GearState> gearStateSupplier) {

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.

    m_gearStateSupplier = gearStateSupplier;

    // Motors
    configmotors();

    // PID values for the talons
    setWheelPIDF();

    m_diffDrive = new DifferentialDrive(m_leftLeader, m_rightLeader);   

    // Feedforward contraints
    // need ????? m_feedForward = DrivetrainConstants.kFeedForward;
        
    // Save previous wheel speeds. Start at zero.
   // need ????  m_prevSpeeds = new DifferentialDriveWheelSpeeds(0,0);
    // old , done in config motors m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    // need ???? m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);
   // need ????  m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) / kCountsPerRevolution);

  // new Zero the encoders and gyro
    resetEncoders();
    zeroGyro();
  }

  public void arcadeDrive(double xaxisSpeed, double zaxisRotate) {
    m_diffDrive.arcadeDrive(xaxisSpeed, zaxisRotate);
  }
    //new
    public void zeroGyro(){
        m_pigeon.reset();
    }

      // new
    public void resetEncoders(){
      m_leftLeader.setSelectedSensorPosition(0);
      m_rightLeader.setSelectedSensorPosition(0);
  }
  // public void resetEncoders() {
  //   m_leftEncoder.reset();
  //   m_rightEncoder.reset();
  // }

  public void setWheelPIDF() {

    // set the PID values for each individual wheel
    for(TalonFX fx : new TalonFX[] {m_leftLeader, m_rightLeader}){
        
        fx.config_kP(0, DrivetrainConstants.kGainsProfiled.kP, 0);
        fx.config_kI(0, DrivetrainConstants.kGainsProfiled.kI, 0);
        fx.config_kD(0, DrivetrainConstants.kGainsProfiled.kD, 0);
        fx.config_kF(0, DrivetrainConstants.kGainsProfiled.kF, 0);
        // m_talonsMaster.config_IntegralZone(0, 30);
    }
}

    public void configmotors() { //new

        // Configure the motors
        for(TalonFX fx : new TalonFX[] {m_leftLeader, m_leftFollower, m_rightLeader, m_rightFollower}){
            //Reset settings for safety
            fx.configFactoryDefault();

            //Sets voltage compensation to 12, used for percent output
            fx.configVoltageCompSaturation(10);
            fx.enableVoltageCompensation(true);

            //Setting just in case
            fx.configNominalOutputForward(0);
            fx.configNominalOutputReverse(0);
            fx.configPeakOutputForward(1);
            fx.configPeakOutputReverse(-1);

            fx.configOpenloopRamp(0.1);

            //Setting deadband(area required to start moving the motor) to 1%
            fx.configNeutralDeadband(0.01);

            //Set to brake mode, will brake the motor when no power is sent
            fx.setNeutralMode(NeutralMode.Coast);

            /** 
             * Setting input side current limit (amps)
             * 45 continious, 80 peak, 30 millieseconds allowed at peak
             * 40 amp breaker can support above 40 amps for a little bit
             * Falcons have insane acceleration so allowing it to reach 80 for 0.03 seconds should be fine
             */
            fx.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 55, 20));

            //Either using the integrated Falcon sensor or an external one, will change if needed
            fx.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor); 
        }
        
        //Setting followers, followers don't automatically followtLeader's inverts so you must set the invert type to FollotLeader
        m_leftFollower.follow(m_leftLeader, FollowerType.PercentOutput);
        m_leftFollower.setInverted(InvertType.FollowMaster);
        m_rightFollower.follow(m_rightLeader, FollowerType.PercentOutput);
        m_rightFollower.setInverted(InvertType.FollowMaster);

        m_rightLeader.setInverted(InvertType.InvertMotorOutput);
    }

    public void setDriveTrainVoltage(double leftVolts, double rightVolts) {
      m_leftLeader.set(ControlMode.PercentOutput, leftVolts/12);
      m_rightLeader.set(ControlMode.PercentOutput, rightVolts/12);
      m_diffDrive.feed();
  }
  public double motorRotationsToWheelRotations(double motorRotations, Transmission.GearState gearState) {
    if (gearState == Transmission.GearState.HIGH) {
        return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kHighGearRatio);
    }
    return motorRotations/(DrivetrainConstants.kEncoderCPR * DrivetrainConstants.kLowGearRatio);
}

public double wheelRotationsToMeters(double wheelRotations) {
  return DrivetrainConstants.kWheelDiameterMeters * Math.PI * wheelRotations;
}
   // Encoder ticks to meters
   public double encoderTicksToMeters(double encoderTicks) {
    var gearState = m_gearStateSupplier.get();
    double wheelRotations = motorRotationsToWheelRotations(encoderTicks, gearState);
    return wheelRotationsToMeters(wheelRotations);
}
  // public int getLeftEncoderCount() {
  //   return m_leftEncoder.get();
  // }

  // public int getRightEncoderCount() {
  //   return m_rightEncoder.get();
  // }

  //
  //public double getLeftDistanceInch() {
    // return m_leftEncoder.getDistance();
  // }

   public double getLeftDistanceMeters() {
    return encoderTicksToMeters(m_leftLeader.getSelectedSensorPosition());
}

public double getRightDistanceMeters() {        
    return encoderTicksToMeters(m_rightLeader.getSelectedSensorPosition());
}
public double getLeftDistanceInch() {
  return getLeftDistanceMeters() * 39.3701;
}

public double getRightDistanceInch() {        
  return getRightDistanceMeters() * 39.3701;
}

public double getAvgDistanceMeters(){
    return (getLeftDistanceMeters() + getRightDistanceMeters()) /2;
}
public double getAverageDistanceInch() {
  return getAvgDistanceMeters() * 39.3701;
}


  // public double getRightDistanceInch() {
  //   return m_rightEncoder.getDistance();
  // }

  // public double getAverageDistanceInch() {
  //   return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  // }

  /**
   * The acceleration in the X-axis.
   *
   * @return The acceleration of the Romi along the X-axis in Gs
   */
  // public double getAccelX() {
  //   return m_accelerometer.getX();
  // }

  /**
   * The acceleration in the Y-axis.
   *
   * @return The acceleration of the Romi along the Y-axis in Gs
   */
  // public double getAccelY() {
  //   return m_accelerometer.getY();
  // }

  /**
   * The acceleration in the Z-axis.
   *
   * @return The acceleration of the Romi along the Z-axis in Gs
   */
  // public double getAccelZ() {
  //   return m_accelerometer.getZ();
  // }

  /**
   * Current angle of the Romi around the X-axis.
   *
   * @return The current angle of the Romi in degrees
   */
  // TODO get from pigeon
  // public double getGyroAngleX() {
  //   return m_gyro.getAngleX();
  // }

  /**
   * Current angle of the Romi around the Y-axis.
   *
   * @return The current angle of the Romi in degrees
   */
    // TODO get from pigeon
  // public double getGyroAngleY() {
  //   return m_gyro.getAngleY();
  // }

  /**
   * Current angle of the Romi around the Z-axis.
   *
   * @return The current angle of the Romi in degrees
   */
    // TODO get from pigeon
// public double getGyroAngleZ() {
//     return m_gyro.getAngleZ();
//   }

  /** Reset the gyro. */
   // TODO get from pigeon
  //  public void resetGyro() {
  //   m_gyro.reset();
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
