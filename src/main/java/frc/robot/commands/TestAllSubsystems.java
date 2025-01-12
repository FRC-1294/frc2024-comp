// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.constants.CompConstants;

public class TestAllSubsystems extends Command {
  /** Creates a new TestAllSubsystems. */
  IntakeSubsystem mIntake;
  AimingSubsystem mAiming;
  LauncherSubsystem mLauncher; 
  SendableChooser<Integer> mChoose = new SendableChooser<>();
public TestAllSubsystems(IntakeSubsystem intake, AimingSubsystem aiming, LauncherSubsystem launcher)
{
    // Use addRequirements() here to declare subsystem dependencies.
    mIntake = intake;
    mAiming = aiming;
    mLauncher = launcher;
    addRequirements(mIntake);
    addRequirements(mAiming);
    addRequirements(mLauncher);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putData("pick the test", mChoose);
    mChoose.addOption("intake", 0);
    mChoose.addOption("laucher", 1);
    mChoose.addOption("wrist", 2);
    mChoose.addOption("elevator", 3);

    mAiming.setDesiredElevatorDistance(0);
    mAiming.setDesiredWristRotation(0); 

    CompConstants.DEBUG_MODE = true;
  }

  // Called every time the scheduler runs which the command is scheduled.
  @Override
  public void execute() {
    switch (mChoose.getSelected()) {
      case 0:
        mIntake.intakeMotorsAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
        if (!mIntake.pieceInIntake()) {
          mIntake.stopMotors();
        }
        SmartDashboard.putBoolean("pieceInIntake", mIntake.pieceInIntake());
        SmartDashboard.putNumber("innerIntakeMotor", mIntake.getIntakeSpeed()[0]);
        SmartDashboard.putNumber("outerIntakeMotor", mIntake.getIntakeSpeed()[1]);
        break;
      
      case 1:
        mLauncher.runLauncher();
        if (!mLauncher.pieceInIndexer()){
          mLauncher.runIndexer(LauncherConstants.INDEXER_VELOCITY_INDEXING);
        } else if (mLauncher.pieceInIndexer() && mLauncher.isLauncherReady()) {
          mLauncher.runLauncher();
        }
        SmartDashboard.putNumber("indexerMotor", mLauncher.getLauncherSpeeds()[0]);
        SmartDashboard.putNumber("leaderMotor", mLauncher.getLauncherSpeeds()[1]);
        SmartDashboard.putNumber("followerMotor", mLauncher.getLauncherSpeeds()[2]);
        SmartDashboard.putBoolean("pieceInIndexer", mLauncher.pieceInIndexer());
        break;
      case 2: 
        mAiming.setDesiredWristRotation(SmartDashboard.getNumber("wristSetpointDegrees", 90));
        break;
      case 3:
        mAiming.setDesiredElevatorDistance(SmartDashboard.getNumber("elevatorSetpointInches", 2));
        break;
      default:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SmartDashboard.getBoolean("endTesting", false);
  }
}
