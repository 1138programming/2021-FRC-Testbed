package frc.robot.CommandGroups;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Robot;
import frc.robot.commands.MoveBaseFor;

public class AutonMoveBaseFor extends SequentialCommandGroup {
  public AutonMoveBaseFor() {
    addCommands(new MoveBaseFor(0.2, 0.2, 0.0, true, 1000));
  }
}