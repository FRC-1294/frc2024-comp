// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.proto.Photon;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.robots.CompetitionBotSwerveConfig;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PhotonCameras;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem mSwerveSubsystem = new SwerveSubsystem(new CompetitionBotSwerveConfig());
  //private final PhotonCameras mCameras = new PhotonCameras();
  private final IntakeSubsystem mIntakeSubsystem = new IntakeSubsystem();
  private final LauncherSubsystem mLauncherSubsystem = new LauncherSubsystem();
  private final AimingSubsystem mAimingSubsystem = new AimingSubsystem();
  
  

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    mSwerveSubsystem.setDefaultCommand(new DefaultDriveCommand(mSwerveSubsystem));
  }
  
  public SwerveSubsystem getSwerveSubsystem() {
      return mSwerveSubsystem;
  }
  
  public IntakeSubsystem getIntakeSubsystem(){
    return mIntakeSubsystem;
  }
  
  public AimingSubsystem getAimingSubsystem(){
    return mAimingSubsystem;
  }

  public LauncherSubsystem getLauncher(){
    return mLauncherSubsystem;
  }
}
