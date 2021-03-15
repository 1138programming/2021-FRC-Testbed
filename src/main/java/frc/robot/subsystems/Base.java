package frc.robot.subsystems;

import java.util.*;
import static frc.robot.Constants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXFeedbackDevice;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.SwerveModuleController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Base extends SubsystemBase {
    //Creating the Talons
    private final TalonFX leftFrontSpeed, leftBackSpeed, rightBackSpeed, rightFrontSpeed;
    private final TalonFX leftFrontAngle, leftBackAngle, rightBackAngle, rightFrontAngle;

    private final TalonSRX leftFrontMagEncoder, leftBackMagEncoder, rightFrontMagEncoder, rightBackMagEncoder;

    private final Translation2d leftFrontLocation, leftBackLocation, rightFrontLocation, rightBackLocation;
    private final SwerveDriveKinematics kinematics; 

    private final SwerveModuleController leftFrontModule, rightFrontModule, leftBackModule, rightBackModule; 
    private SwerveModuleController[] modules;

    public Base() {
        // Instantiating the Talons
        leftFrontSpeed = new TalonFX(KLeftFrontSpeedTalon);
        leftBackSpeed = new TalonFX(KLeftBackSpeedTalon);
        rightFrontSpeed = new TalonFX(KRightFrontSpeedTalon);
        rightBackSpeed = new TalonFX(KRightBackSpeedTalon);

        leftFrontAngle = new TalonFX(KLeftFrontAngleTalon);
        leftBackAngle = new TalonFX(KLeftBackAngleTalon);
        rightFrontAngle = new TalonFX(KRightFrontAngleTalon);
        rightBackAngle = new TalonFX(KRightBackAngleTalon);

        leftFrontSpeed.setNeutralMode(NeutralMode.Brake);
        leftBackSpeed.setNeutralMode(NeutralMode.Brake);
        rightFrontSpeed.setNeutralMode(NeutralMode.Brake);
        rightBackSpeed.setNeutralMode(NeutralMode.Brake);

        leftFrontAngle.setNeutralMode(NeutralMode.Brake);
        leftBackAngle.setNeutralMode(NeutralMode.Brake);
        rightFrontAngle.setNeutralMode(NeutralMode.Brake);
        rightBackAngle.setNeutralMode(NeutralMode.Brake);

        //Mag Encoder
        leftFrontMagEncoder = new TalonSRX(KLeftFrontMagEncoder);
        leftBackMagEncoder = new TalonSRX(KLeftBackMagEncoder);
        rightFrontMagEncoder = new TalonSRX(KRightFrontMagEncoder);
        rightBackMagEncoder = new TalonSRX(KRightBackMagEncoder);

        leftFrontMagEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        leftBackMagEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        rightFrontMagEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);
        rightBackMagEncoder.configSelectedFeedbackSensor(TalonSRXFeedbackDevice.CTRE_MagEncoder_Absolute, 0, 0);

        // TODO: Locations for the swerve drive modules relative to the robot center
        leftFrontLocation = new Translation2d(0.5207, 0.5207);
        leftBackLocation = new Translation2d(-0.5207, 0.5207);
        rightFrontLocation = new Translation2d(0.5207, -0.5207);
        rightBackLocation = new Translation2d(-0.5207, -0.5207);

        kinematics = new SwerveDriveKinematics(leftFrontLocation, rightFrontLocation, leftBackLocation, rightBackLocation);
        
        //modules
        leftFrontModule = new SwerveModuleController(leftFrontSpeed, leftFrontAngle, leftFrontMagEncoder, Rotation2d.fromDegrees(0));
        leftBackModule = new SwerveModuleController(leftBackSpeed, leftBackAngle, leftBackMagEncoder, Rotation2d.fromDegrees(0));
        rightFrontModule = new SwerveModuleController(rightFrontSpeed, rightFrontAngle, rightFrontMagEncoder, Rotation2d.fromDegrees(0));
        rightBackModule = new SwerveModuleController(rightBackSpeed, rightBackAngle, rightBackMagEncoder, Rotation2d.fromDegrees(0));

        modules = new SwerveModuleController[] {
            leftFrontModule, leftBackModule, rightFrontModule, rightBackModule
        };
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        SwerveModuleState[] states =
          kinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()));
        SwerveDriveKinematics.normalizeWheelSpeeds(states, KBasePWM);
        for (int i = 0; i < states.length; i++) {
          SwerveModuleController module = modules[i];
          SwerveModuleState state = states[i];
          module.setDesiredState(state);
        }
    }

    @Override
    public void periodic() {
    }
  
    @Override
    public void simulationPeriodic() {
    }
}
