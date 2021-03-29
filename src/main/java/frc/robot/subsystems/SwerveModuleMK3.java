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
public class SwerveModuleMK3 {

  private static final Gains kDriveGains = new Gains(15, 0.01, 0.1, 0.2, 0, 1.0);
  private static final Gains kAngleGains = new Gains(1.0, 0.0, 0.0, 0.0, 0, 1.0);

  // CANCoder has 4096 ticks/rotation
  private static double kEncoderTicksPerRotation = 4096;

  private static double desiredTicks;

  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANifier canifier;
  private Rotation2d offset;
  private Boolean invert;

  public SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor, CANifier canifier, Rotation2d offset, Boolean invert) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canifier = canifier;
    this.offset = offset;
    this.invert = invert;

    //angleMotor.configAllowableClosedloopError(0, 0, 0);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		angleMotor.config_kP(0, kAngleGains.kP, 0);
		angleMotor.config_kI(0, kAngleGains.kI, 0);
    angleMotor.config_kD(0, kAngleGains.kD, 0);
    
    angleMotor.setNeutralMode(NeutralMode.Brake); //not needed but nice to keep the robot stopped when you want it stopped

    angleMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

    //driveMotor.configAllowableClosedloopError(0, 0, 0);

		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		driveMotor.config_kF(0, kDriveGains.kF, 0);
		driveMotor.config_kP(0, kDriveGains.kP, 0);
		driveMotor.config_kI(0, kDriveGains.kI, 0);
    driveMotor.config_kD(0, kDriveGains.kD, 0);

    driveMotor.setInverted(invert);
    driveMotor.setNeutralMode(NeutralMode.Brake);
  }


  /**
   * Gets the relative rotational position of the module
   * @return The relative rotational position of the angle motor in degrees
   */
  public Rotation2d getAngle() {
    double deg = canifier.getQuadraturePosition() * 360.0 / 4096.0;

    deg *= 10;
		deg = (int) deg;
    deg /= 10;
    
    return Rotation2d.fromDegrees(deg); //include angle offset
  }
  public double getRawAngle() {
    double deg = canifier.getQuadraturePosition() * 360.0 / 4096.0;

    deg *= 10;
		deg = (int) deg;
    deg /= 10;

    return deg; //include angle offset
  }

  public double getDesiredTicks() {
    return desiredTicks;
  }

  public void zeroEncoders() {
    angleMotor.setSelectedSensorPosition(0, 0, 0);
    canifier.setQuadraturePosition(0, 0);
  }
  //:)
  /**
   * Set the speed + rotation of the swerve module from a SwerveModuleState object
   * @param desiredState - A SwerveModuleState representing the desired new state of the module
   */
  public void setDesiredState(SwerveModuleState desiredState) {

    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    
    // Find the difference between our current rotational position + our new rotational position
    Rotation2d rotationDelta = state.angle.minus(currentRotation);
    
    // Find the new absolute position of the module based on the difference in rotation
    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    // Convert the CANCoder from it's position reading back to ticks
    double currentTicks = getRawAngle() / 0.087890625;
    desiredTicks = deltaTicks;

    //below is a line to comment out from step 5
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);

    double feetPerSecond = Units.metersToFeet(state.speedMetersPerSecond)/2;

    //below is a line to comment out from step 5
    driveMotor.set(TalonFXControlMode.PercentOutput, feetPerSecond / kMaxSpeed);
  }

}
