package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.SlewRateLimiter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class SwerveWithJoysticks extends CommandBase {
    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  public SwerveWithJoysticks() {
    addRequirements(Robot.base);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      final double xSpeed = -xspeedLimiter.calculate(KBasePWM * Robot.m_robotContainer.getLeftAxis());
      final double ySpeed = -yspeedLimiter.calculate(KBasePWM * Robot.m_robotContainer.getArcadeLeftAxis());
      final double rot = -rotLimiter.calculate(KBasePWM * Robot.m_robotContainer.getArcadeRightAxis());

      base.drive(xSpeed, ySpeed, rot);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}