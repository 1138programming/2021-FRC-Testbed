package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import static frc.robot.Constants.*;

import java.time.Duration;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class MoveBaseFor extends CommandBase {
    private double xSpeed, ySpeed, rot;
    private boolean fieldRelative;
    private long duration;
    private long startTime;

    public MoveBaseFor(double xSpeed, double ySpeed, double rot, boolean fieldRelative, long duration)  {
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rot = rot;
        this.fieldRelative = fieldRelative;
        this.duration = duration;

        addRequirements(Robot.base);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Robot.base.drive(xSpeed, ySpeed, rot, fieldRelative);;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        Robot.base.drive(0, 0, 0, true);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (duration != 0) && (System.currentTimeMillis() - startTime) > duration;
    }
}
