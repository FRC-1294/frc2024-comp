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

public class SwerveVoltageCharacterization extends Command {
  /** Creates a new kV_Characterization. */
  private final SwerveSubsystem mSwerve;
  private final SwerveModuleAbstract[] mModules;
  private final Timer mTimer = new Timer();
  private final double mTargVelMPS;
  private double mIncrement = 0;
  private double[] mVelocityConversion;
  private double mSampleNo = 0;
  private boolean mTimerHasStarted = false;

  public SwerveVoltageCharacterization(SwerveSubsystem swerve) {
    mSwerve = swerve;
    mModules = mSwerve.getRawModules();
    mTargVelMPS = 3;
    addRequirements(mSwerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mVelocityConversion = new double[mSwerve.mConfig.NUM_MODULES];
    mTimer.reset();
    mSwerve.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Math.abs(SwerveSubsystem.getChassisSpeeds().vxMetersPerSecond - mTargVelMPS) < 0.1){
        if (!mTimerHasStarted){
            mTimer.start();
        }

        mSampleNo += 1;
        for(int i = 0; i<mModules.length; i++){
            double curkV = (mModules[i].getTransAppliedVolts()/mModules[i].getTransNominalVoltage())/mModules[i].getTransVelocity();
            mVelocityConversion[i] = (mVelocityConversion[i]*(mSampleNo-1)+curkV)/mSampleNo;
            SmartDashboard.putNumber("avgkV"+i, mVelocityConversion[i]);
        }
    }else{
      mSwerve.setChassisSpeed(mIncrement, 0, 0,true,true);
      mIncrement +=0.016;
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
    return mTimer.get()>2;
  }
}
