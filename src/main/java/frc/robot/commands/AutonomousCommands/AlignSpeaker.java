// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutonomousCommands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class AlignSpeaker extends Command {
  /** Creates a new AlignSpeaker. */
  private final SwerveSubsystem mSwerve;
  public AlignSpeaker(SwerveSubsystem swerve) {
    mSwerve = swerve;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mSwerve.setChassisSpeed(0, 0, Math.toRadians(
      DefaultDriveCommand.mSpeakerAlignPID.calculate(
        SwerveSubsystem.getRobotPose().getRotation().getDegrees(),
        DefaultDriveCommand.getRotationToSpeakerDegrees())),true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.setChassisSpeed(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return DefaultDriveCommand.getAlignedToSpeaker();
  }
}
