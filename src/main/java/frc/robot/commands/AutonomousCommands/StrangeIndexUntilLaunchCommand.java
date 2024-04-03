// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.LauncherSubsystem;

public class StrangeIndexUntilLaunchCommand extends Command {

  Command reverse;
  LauncherSubsystem mLauncherSubsystem;
  boolean currentPassThrough = false;
  boolean achievedBackness = false;

  /** Creates a new StrangeIndexUntilLaunchCommand. */
  public StrangeIndexUntilLaunchCommand(LauncherSubsystem launcherSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    mLauncherSubsystem = launcherSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    DefaultMechCommand.isLaunching = true;
    currentPassThrough = false;
    achievedBackness  = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (!currentPassThrough && mLauncherSubsystem.pieceInIndexer()){
      mLauncherSubsystem.runIndexer(-0.15);
      currentPassThrough = true;
    }
    if (!mLauncherSubsystem.pieceInIndexer() && currentPassThrough){
      achievedBackness = true;
      mLauncherSubsystem.runIndexer(.3);
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // mLauncherSubsystem.stopIndexer();
    SmartDashboard.putBoolean("Ended", true);
  }
  

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (currentPassThrough && mLauncherSubsystem.pieceInIndexer() && achievedBackness ) || FieldConstants.getSpeakerDistance() < 1.6; 
  }
}
