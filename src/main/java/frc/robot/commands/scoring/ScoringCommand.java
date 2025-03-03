package frc.robot.commands.scoring;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class ScoringCommand extends Command {
  private final ScoringSubsystem scoringSystem;
  //private final int pushPullFactor; // 0 = nothing, 1 = push, -1 = pull

  public ScoringCommand(ScoringSubsystem system, double target, int pushPullFactor)
  {
    this.scoringSystem = system;
    //this.pushPullFactor = pushPullFactor;
    addRequirements(scoringSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Scoring subystem moving");
  }

    // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    scoringSystem.moveToPosition(ScoringSubsystem.globalTargetRotations);

    /* Do I need this code? Not sure.
    if(pushPullFactor == 1)
    {
      scoringSystem.launch();
    }
    else if (pushPullFactor == -1)
    {
      scoringSystem.pull();
    }
    */
  }
  
    
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Scoring subsystem movement ended");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // leave false, so always can move
    return false;
  }
}
