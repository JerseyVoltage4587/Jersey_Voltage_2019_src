package frc.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.Robot;

public class Hab2Left2Hatch extends CommandGroup {
  
  public Hab2Left2Hatch() {
    addSequential(new FollowPath(Robot.getPathManager().getTrajMap().get("hab2ToLeftNearCargoLeft"), Robot.getPathManager().getTrajMap().get("hab2ToLeftNearCargoRight")));
    //addSequential(new StartSimpleVision());
    //addSequential(new FollowPath(Robot.getPathManager().getTrajMap().get("leftNearCargoToLoadingLeft"), Robot.getPathManager().getTrajMap().get("leftNearCargoToLoadingRight")));
    //addSequential(new FollowPath(Robot.getPathManager().getTrajMap().get("loadingToLeftMiddleCargoLeft"), Robot.getPathManager().getTrajMap().get("loadingToLeftMiddleCargoRight")));
  
  }
}
