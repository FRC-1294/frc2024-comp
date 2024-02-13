// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.swerve.SwerveModuleAbstract;

public class KvCharacterization extends Command {
  /** Creates a new kV_Characterization. */
  private final SwerveSubsystem mSwerve;
  private final SwerveModuleAbstract[] mModules;
  private final Timer mTimer = new Timer();
  private final double mTargVelMPS;
  private double increment = 0;

  private double [] expkV = new double[4];
  private double sampleNo = 0;
  private boolean timerHasStarted = false;

  public KvCharacterization(SwerveSubsystem swerve) {
    mSwerve = swerve;
    mModules = mSwerve.getRawModules();
    mTargVelMPS = 1;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTimer.reset();
    mSwerve.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(SwerveSubsystem.getChassisSpeeds().vxMetersPerSecond - mTargVelMPS)<0.1){
        if (!timerHasStarted){
            mTimer.start();
        }

        sampleNo += 1;
        for(int i = 0; i<4; i++){
            double curkV = (mModules[i].getTransAppliedVolts()/mModules[i].getTransNominalVoltage())/mModules[i].getTransVelocity();
            expkV[i] = (expkV[i]*(sampleNo-1)+curkV)/sampleNo;
            SmartDashboard.putNumber("avgkV"+i, expkV[i]);
        }
    }else{
      mSwerve.setChassisSpeed(increment, 0, 0,false,true);
      increment +=0.01;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerve.setChassisSpeed(new ChassisSpeeds(0,0,0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTimer.get()>4;
  }
}
