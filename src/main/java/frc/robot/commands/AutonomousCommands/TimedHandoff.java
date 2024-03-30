// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class TimedHandoff extends Command {
  /** Creates a new Handofff. */
  private final IntakeSubsystem mIntake;
  private final LauncherSubsystem mLauncher;
  private Timer time = new Timer();
  public TimedHandoff(IntakeSubsystem intake, LauncherSubsystem launcher) {
    mIntake = intake;
    mLauncher = launcher;
    addRequirements(mIntake,mLauncher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    time.reset();
    time.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //accelerates intake if it runs for too long
    if (time.get() > 0.5) {
      mIntake.innerMotorAtSpeed(Math.min(IntakeConstants.INNER_INTAKE_SPEED_HANDOFF + time.get() * IntakeConstants.HANDOFF_TIMER_MULTIPLIER, 1));
      mLauncher.runIndexer(Math.min(LauncherConstants.INDEXER_VELOCITY_INDEXING + time.get() * IntakeConstants.HANDOFF_TIMER_MULTIPLIER, 1));
    }
    else {
      mIntake.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_HANDOFF);
      mLauncher.runIndexer(LauncherConstants.INDEXER_VELOCITY_INDEXING);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mLauncher.stopIndexer();
    mIntake.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mLauncher.pieceInIndexer() && !IntakeSubsystem.pieceInIntake();
  }
}
