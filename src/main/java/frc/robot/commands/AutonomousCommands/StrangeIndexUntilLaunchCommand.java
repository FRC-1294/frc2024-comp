// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;

public class StrangeIndexUntilLaunchCommand extends Command {

  Command reverse;
  LauncherSubsystem mLauncherSubsystem;
  boolean didPassThrough = false;

  /** Creates a new StrangeIndexUntilLaunchCommand. */
  public StrangeIndexUntilLaunchCommand(LauncherSubsystem launcherSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mLauncherSubsystem = launcherSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    reverse = mLauncherSubsystem.reverse();
    DefaultMechCommand.isLaunching = true;
    new SequentialCommandGroup(
      reverse,
      new InstantCommand(() -> mLauncherSubsystem.runIndexer(LauncherConstants.INDEXER_VELOCITY_LAUNCHING))
    ).schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mLauncherSubsystem.pieceInIndexer() && reverse.isFinished()){
      didPassThrough = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    DefaultMechCommand.isLaunching = false;
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return didPassThrough && !mLauncherSubsystem.pieceInIndexer();
  }
}
