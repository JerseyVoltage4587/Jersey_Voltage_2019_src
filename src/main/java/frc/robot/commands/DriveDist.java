package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class DriveDist extends Command {
  double m_dist;
  public DriveDist(double dist) {
    m_dist = dist;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.getDrive().setDesiredDist(m_dist);
    Robot.getDrive().startDriveDist();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Robot.getDrive().getDriveDone();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}