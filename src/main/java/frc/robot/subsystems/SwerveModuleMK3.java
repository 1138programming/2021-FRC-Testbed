package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierConfiguration;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Gains;

import edu.wpi.first.wpilibj.controller.PIDController;

public class SwerveModuleMK3 {

  private static double maxDeltaTicks = 990.0;

  private static final Gains kDriveGains = new Gains(15, 0.01, 0.1, 0.2, 0, 1.0);
  // private static final Gains kAngleGains = new Gains(0.0, 0.0, 0.0, 0.0, 0, 0.0); //angle gains: 1.0, 0.0, 0.0, 0.0, 0, 1.0

  // CANCoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private double desiredTicks;

  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANifier canifier;
  private Rotation2d offset;

  private double calculatedDesireTicks;
  //private Boolean invert;

  private PIDController angleMotorController;

  public SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor, CANifier canifier, Rotation2d offset, PIDController angleMotorPIDController) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canifier = canifier;
    this.offset = offset;
    this.angleMotorController = angleMotorPIDController;
    //this.invert = invert;

    //angleMotor.configAllowableClosedloopError(0, 0, 0);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		// angleMotor.config_kP(0, kAngleGains.kP, 0);
		// angleMotor.config_kI(0, kAngleGains.kI, 0);
    // angleMotor.config_kD(0, kAngleGains.kD, 0);
    
    angleMotor.setNeutralMode(NeutralMode.Brake); //not needed but nice to keep the robot stopped when you want it stopped

    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

    //driveMotor.configAllowableClosedloopError(0, 0, 0);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		driveMotor.config_kF(0, kDriveGains.kF, 0);
		driveMotor.config_kP(0, kDriveGains.kP, 0);
		driveMotor.config_kI(0, kDriveGains.kI, 0);
    driveMotor.config_kD(0, kDriveGains.kD, 0);

    //driveMotor.setInverted(invert);
    driveMotor.setNeutralMode(NeutralMode.Brake);
  }


  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / ticksPerRevolution;
    //double deg = canifier.getQuadraturePosition() * 360.0 / 4096.0;
    if (deg < 0){
      deg = deg +360;
    }
    //deg *= 10;
		//deg = (int) deg;
    //deg /= 10;
    
    return Rotation2d.fromDegrees(deg); //include angle offset
  }
  public double getRawAngle() {
    //double deg = canifier.getQuadraturePosition() * 360.0 / 4096.0;

    double deg = (canifier.getQuadraturePosition() % ticksPerRevolution) * 360 / ticksPerRevolution;

    if (deg < 0){
      deg = deg +360;
    }

    //deg *= 10;
		//deg = (int) deg;
    //deg /= 10;

    return deg; //include angle offset
  }

  public double getCurrentTicks() {
    return canifier.getQuadraturePosition();
  }

  public double getDesiredTicks() {
    return desiredTicks;
  }

  public void zeroEncoders() {
    angleMotor.setSelectedSensorPosition(0, 0, 0);
    canifier.setQuadraturePosition(0, 0);
  }
  
  public void resetAngleSetpoint() {
    angleMotor.set(TalonFXControlMode.Position, 0);
  }

  public double getSetpoint() {
    return calculatedDesireTicks;
  }
  //:)
  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d getAngle = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, getAngle);
    
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(getAngle);
    
    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = canifier.getQuadraturePosition();

    desiredTicks = deltaTicks;

    if (desiredTicks > maxDeltaTicks) {
      desiredTicks = maxDeltaTicks;
    } else if (desiredTicks < -maxDeltaTicks) {
      desiredTicks = -maxDeltaTicks;
    }

    //below is a line to comment out from step 5
    calculatedDesireTicks = angleMotorController.calculate(canifier.getQuadraturePosition(), desiredTicks);
    angleMotor.set(TalonFXControlMode.Position, calculatedDesireTicks);

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond)/2;

    //below is a line to comment out from step 5
    //driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / kMaxSpeed);
  }
  public void setAnglePIDGains(double kP, double kI, double kD){
    angleMotorController.setP(kP);
    angleMotorController.setI(kI);
    angleMotorController.setD(kD);
  }
}
