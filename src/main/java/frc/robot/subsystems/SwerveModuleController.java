package frc.robot.subsystems;

import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.util.Units;

public class SwerveModuleController {
    // TODO: Tune these PID values for your robot
    private static final double kDriveP = 15.0;
    private static final double kDriveI = 0.01;
    private static final double kDriveD = 0.1;
    private static final double kDriveF = 0.2;

    private static final double kAngleP = 1.0;
    private static final double kAngleI = 0.0;
    private static final double kAngleD = 0.0;

    // CANCoder has 4096 ticks/rotation
    private static double kEncoderTicksPerRotation = 4096;

    private TalonFX driveMotor;
    private TalonFX angleMotor;
    private TalonSRX magEncoder;

    private TalonFXConfiguration angleTalonFXConfiguration;
    private TalonFXConfiguration driveTalonFXConfiguration;
    private TalonSRXConfiguration magEncoderConfiguration;

    public SwerveModuleController(TalonFX driveMotor, TalonFX angleMotor, TalonSRX magEncoder, Rotation2d offset) {
        this.driveMotor = driveMotor;
        this.angleMotor = angleMotor;
        this.magEncoder = magEncoder;

        angleTalonFXConfiguration = new TalonFXConfiguration();

        angleTalonFXConfiguration.slot0.kP = kAngleP;
        angleTalonFXConfiguration.slot0.kI = kAngleI;
        angleTalonFXConfiguration.slot0.kD = kAngleD;

        angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = magEncoder.getDeviceID();
        angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.TalonSRX_SelectedSensor;
        angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Absolute;
        angleMotor.configAllSettings(angleTalonFXConfiguration);

        driveTalonFXConfiguration = new TalonFXConfiguration();

        driveTalonFXConfiguration.slot0.kP = kDriveP;
        driveTalonFXConfiguration.slot0.kI = kDriveI;
        driveTalonFXConfiguration.slot0.kD = kDriveD;
        driveTalonFXConfiguration.slot0.kF = kDriveF;

        driveMotor.configAllSettings(driveTalonFXConfiguration);

        magEncoderConfiguration = new TalonSRXConfiguration();
        //magEncoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
        magEncoder.configAllSettings(magEncoderConfiguration);
    }


    /**
     * Gets the relative rotational position of the module
     * @return The relative rotational position of the angle motor in degrees
     */
    public Rotation2d getAngle() {
        // Note: This assumes the CANCoders are setup with the default feedback coefficient
        // and the sesnor value reports degrees.
        double deg = magEncoder.getSelectedSensorPosition() * 360.0 / 4096.0;

        deg *= 10;
		deg = (int) deg;
        deg /= 10;
        
        return Rotation2d.fromDegrees(deg);
    }

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
        double currentTicks = magEncoder.getSelectedSensorPosition();
        double desiredTicks = currentTicks + deltaTicks;
        angleMotor.set(TalonFXControlMode.Position, desiredTicks);

        double meterPerSecond = state.speedMetersPerSecond;
        driveMotor.set(TalonFXControlMode.PercentOutput, meterPerSecond / KBasePWM);
    }
}
