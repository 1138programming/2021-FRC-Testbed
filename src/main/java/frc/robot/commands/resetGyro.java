package frc.robot.commands;

import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class ResetGyro extends CommandBase {
    
    public ResetGyro() {
        addRequirements(Robot.base);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }
  
    @Override
    public void execute() {
        Robot.base.resetGyro();    
    }
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return true;
    }
}
