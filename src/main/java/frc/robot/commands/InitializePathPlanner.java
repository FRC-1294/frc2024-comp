// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Map;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutonomousCommands.AlignSpeaker;
import frc.robot.commands.AutonomousCommands.TimedHandoff;
import frc.robot.constants.AimState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class InitializePathPlanner{
  /** Creates a new InitializePathPlanner. */
  private final SwerveSubsystem mSwerve;
  private IntakeSubsystem mIntake;
  private LauncherSubsystem mLauncher;
  private AimingSubsystem mAiming;
  private boolean populateEventMaps;
  public InitializePathPlanner(SwerveSubsystem swerve, IntakeSubsystem intake, LauncherSubsystem launcher, AimingSubsystem aiming) {
    mSwerve = swerve;
    mIntake = intake;
    mLauncher = launcher;
    mAiming = aiming;
    populateEventMaps = true;
  }
  public InitializePathPlanner(SwerveSubsystem swerve){
    mSwerve = swerve;
    populateEventMaps = false;
  }
  
  public void initializeNamedCommands(){
    NamedCommands.registerCommand("IntakeUntilNote", mIntake.getAutomousIntakeCommand());

    NamedCommands.registerCommand("ShootFromMidnote", new SequentialCommandGroup(new SelectCommand<>(Map.ofEntries(
      Map.entry(true, new TimedHandoff(mIntake,mLauncher).withTimeout(2.5)),
      Map.entry(false, new PrintCommand("Failed to pick up Note"))
    ),()->IntakeSubsystem.pieceInIntake()),
      new ParallelCommandGroup(
        mAiming.waitUntilSetpoint(AimState.PODIUM), 
        mLauncher.waitUntilFlywheelSetpointCommand(AimState.PODIUM)), 
      mLauncher.indexUntilNoteLaunchedCommand()));

    NamedCommands.registerCommand("PutShot", new SequentialCommandGroup(new SelectCommand<>(Map.ofEntries(
      Map.entry(true, new TimedHandoff(mIntake,mLauncher).withTimeout(2.5)),
      Map.entry(false, new PrintCommand("Failed to pick up Note"))
    ),()->IntakeSubsystem.pieceInIntake()),
      new ParallelCommandGroup(
        mAiming.waitUntilSetpoint(AimState.SUBWOOFER), 
        mLauncher.waitUntilFlywheelSetpointCommand(AimState.HANDOFF)), 
      mLauncher.indexUntilNoteLaunchedCommand()));

    NamedCommands.registerCommand("ShootFromSubwoofer", new SequentialCommandGroup(
      new SelectCommand<>(Map.ofEntries(
        Map.entry(true, new TimedHandoff(mIntake,mLauncher).withTimeout(2.5)),
        Map.entry(false, new PrintCommand("Failed to pick up Note"))
      ),()->IntakeSubsystem.pieceInIntake()),
      new ParallelCommandGroup(
        mAiming.waitUntilSetpoint(AimState.SUBWOOFER), 
        mLauncher.waitUntilFlywheelSetpointCommand(AimState.SUBWOOFER)).withTimeout(3), 
      mLauncher.indexUntilNoteLaunchedCommand()));

    NamedCommands.registerCommand("ShootFromWing", new SequentialCommandGroup(
      new SelectCommand<>(Map.ofEntries(
        Map.entry(true, new TimedHandoff(mIntake,mLauncher).withTimeout(2.5)),
        Map.entry(false, new PrintCommand("Failed to pick up Note"))
      ),()->IntakeSubsystem.pieceInIntake()),
      new ParallelCommandGroup(
        mAiming.waitUntilSetpoint(AimState.WING), 
        mLauncher.waitUntilFlywheelSetpointCommand(AimState.WING)), 
      mLauncher.indexUntilNoteLaunchedCommand()));

    NamedCommands.registerCommand("ShootFromLine", new SequentialCommandGroup(
      new SelectCommand<>(Map.ofEntries(
        Map.entry(true, new TimedHandoff(mIntake,mLauncher).withTimeout(2.5)),
        Map.entry(false, new PrintCommand("Failed to pick up Note"))
      ),()->IntakeSubsystem.pieceInIntake()),
      new ParallelCommandGroup(
        mAiming.waitUntilSetpoint(AimState.LINE), 
        mLauncher.waitUntilFlywheelSetpointCommand(AimState.LINE)), 
      mLauncher.indexUntilNoteLaunchedCommand()));

    NamedCommands.registerCommand("ShootDynamic", new SequentialCommandGroup(
      new SelectCommand<>(Map.ofEntries(
        Map.entry(true, new TimedHandoff(mIntake,mLauncher).withTimeout(2.5)),
        Map.entry(false, new PrintCommand("Failed to pick up Note"))
      ),()->IntakeSubsystem.pieceInIntake()),
      new ParallelCommandGroup(
            // new AlignSpeaker(mSwerve),
            mLauncher.waitUntilFlywheelSetpointCommand(AimState.PODIUM),
            mAiming.waitUntilAutoAimSetpoint()
      ), 
      mLauncher.indexUntilNoteLaunchedCommand()));

    NamedCommands.registerCommand("StartLauncherSW", mLauncher.waitUntilFlywheelSetpointCommand(AimState.SUBWOOFER));
    NamedCommands.registerCommand("StartLauncherAutoAim", mLauncher.waitUntilFlywheelSetpointCommand(AimState.PODIUM));
    NamedCommands.registerCommand("AutoAimWristSetpoint", mAiming.waitUntilAutoAimSetpoint());
    NamedCommands.registerCommand("Handoff", new SequentialCommandGroup(mAiming.waitUntilSetpoint(AimState.HANDOFF), 
                                        new TimedHandoff(mIntake,mLauncher)));
    NamedCommands.registerCommand("HandoffSetpoint", mAiming.waitUntilSetpoint(AimState.HANDOFF));
    NamedCommands.registerCommand("AlignToSpeaker", new AlignSpeaker(mSwerve));

  }

  public void initializeEmptyNamedCommands(){
    NamedCommands.registerCommand("IntakeUntilNote", new SequentialCommandGroup(new PrintCommand("Intaking until note enters")));
    NamedCommands.registerCommand("ShootFromMidnote", new SequentialCommandGroup(new PrintCommand("Shoot Note from Middle Position"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("ShootFromSubwoofer", new SequentialCommandGroup(new PrintCommand("Shoot Note From Subwoofer"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("ShootFromWing", new SequentialCommandGroup(new PrintCommand("Shoot Note from Wing Edge"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("ShootFromLine", new SequentialCommandGroup(new PrintCommand("Shoot Note from Autonomous Line"), new WaitCommand(0.5)));
    NamedCommands.registerCommand("Handoff", new SequentialCommandGroup(new PrintCommand("Shoot Note from Wing Edge")));
    NamedCommands.registerCommand("HandoffSetpoint", new PrintCommand("ojdo"));
    NamedCommands.registerCommand("StartLauncherSW", new PrintCommand("oj"));
    NamedCommands.registerCommand("StartWristSW", new PrintCommand("osjod"));
    NamedCommands.registerCommand("PutShot", new PrintCommand("jo"));

  }


  // Called when the command is initially scheduled.
  public void initialize() {
    AutoBuilder.configureHolonomic(
    SwerveSubsystem::getRobotPose, 
    mSwerve::resetRobotPose,
    SwerveSubsystem::getChassisSpeeds, 
    mSwerve::setChassisSpeed,
    new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in
                                      // your Constants class
          new PIDConstants(5, 0.0, 0), // Translation PID constants
          new PIDConstants(5, 0.0, 0), // Rotation PID constants
          4.5, // Max module speed, in m/s
          0.4669, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig(true, false) // Default path replanning config. See the API for the options
                              // here
              
    ), this::determineFieldOrientation, mSwerve);
    if (populateEventMaps){
      initializeNamedCommands();
    } else{
      initializeEmptyNamedCommands();
    }

  }

  public boolean determineFieldOrientation(){
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      return alliance.get() == DriverStation.Alliance.Red;
    }
    return false;
  }
}
