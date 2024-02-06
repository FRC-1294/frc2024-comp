// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CompConstants;
import frc.robot.robots.PracticeBotSwerveConfig;
import frc.robot.robots.SwerveConfig;
import frc.robot.swerve.SwerveModuleAbstract;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private static SwerveDriveKinematics mKinematics;
  private static SwerveDriveOdometry mOdometry;

  private static Pigeon2 mPigeon2;

  private static SwerveModuleAbstract[] mModules;
  private double mTargetSpeed = 0;
  private double mAvgSpeed = 0;
  private double maxSpeed = 0;
  private PIDController chassisRotPID =  new PIDController(0.1, 0, 0);
  private PIDController chassisXPID = new PIDController(0.1, 0, 0);
  private PIDController chassisYPID = new PIDController(0.3, 0, 0);
  public final SwerveConfig mConfig;
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();


  public SwerveSubsystem(SwerveConfig configuration) {
    // Populating Instance Variables

    chassisRotPID.setTolerance(0.2);
    chassisXPID.setTolerance(0.2);
    chassisYPID.setTolerance(0.2);
    
    mConfig = configuration;
    mKinematics = mConfig.SWERVE_KINEMATICS;
    mPigeon2 = mConfig.PIGEON;
    mModules = mConfig.SWERVE_MODULES;
    mOdometry = new SwerveDriveOdometry(mKinematics, getRotation2d(), getModulePositions());
    resetGyro();
    resetRobotPose();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double ts4 = ((double) System.nanoTime())/1000000;
    
    mOdometry.update(getRotation2d(), getModulePositions());
    ////System.out.println("Update Odom Nano: " + ((double)System.nanoTime()/1000000-ts4));
      SmartDashboard.putNumber("XPos", mOdometry.getPoseMeters().getX());
      SmartDashboard.putNumber("YPos", mOdometry.getPoseMeters().getY());
      SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());
    if (CompConstants.DEBUG_MODE) {
      SmartDashboard.putData("Swerve", this);

      SmartDashboard.putNumber("DesiredChassisRotDeg", Math.toDegrees(desiredChassisSpeeds.omegaRadiansPerSecond));
      SmartDashboard.putNumber("DesiredChassisXMPS", desiredChassisSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("DesiredChassisYMPS", desiredChassisSpeeds.vyMetersPerSecond);

      SmartDashboard.putNumber("CurrentChassisRotDeg", Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond));
      SmartDashboard.putNumber("CurrentChassisXMPS", getChassisSpeeds().vxMetersPerSecond);
      SmartDashboard.putNumber("CurrentChassisYMPS", getChassisSpeeds().vyMetersPerSecond);

      SmartDashboard.putNumber("ChassisSpeedErrorRotDeg", Math.toDegrees(desiredChassisSpeeds.omegaRadiansPerSecond-getChassisSpeeds().omegaRadiansPerSecond));
      SmartDashboard.putNumber("ChassisSpeedErrorXMPS", desiredChassisSpeeds.vxMetersPerSecond-getChassisSpeeds().vxMetersPerSecond);
      SmartDashboard.putNumber("ChassisSpeedErrorYMPS", desiredChassisSpeeds.vyMetersPerSecond-getChassisSpeeds().vyMetersPerSecond);

      SmartDashboard.putNumber("FPGA_TS",Timer.getFPGATimestamp());
      for (int i = 0; i < mModules.length; i++) {
        if (mModules[i].getTransVelocity()>maxSpeed){
          maxSpeed = mModules[i].getTransVelocity();
        }
        SmartDashboard.putNumber("MaxSpeed", maxSpeed);
        SmartDashboard.putNumber("TransDistance"+i, mModules[i].getTransPosition());
        SmartDashboard.putNumber("TransAppliedOutput" + i, mModules[i].getTransAppliedVolts());
        mAvgSpeed += Math.abs(mModules[i].getTransVelocity());
        SmartDashboard.putNumber("RotRelativePosDeg" + i,
            mModules[i].getRotRelativePosition() * 360);
        SmartDashboard.putNumber("AbsEncoderDeg" + i, mModules[i].getRotPosition() / Math.PI * 180);
        SmartDashboard.putNumber("TranslationSpeedMeters" + i, mModules[i].getTransVelocity());
        SmartDashboard.putNumber("TranslationDesiredVel" + i, mModules[i].getTransVelocitySetpoint());
        SmartDashboard.putNumber("MaxAccel", i);
      }

      mAvgSpeed = mAvgSpeed / 4;
      SmartDashboard.putNumber("AvgVelocity", mAvgSpeed);
      SmartDashboard.putNumber("TargetVelocity", mTargetSpeed); 
      for (int i = 0; i < mModules.length; i++) {
        SmartDashboard.putNumber("VelocityDeviation" + i,
        Math.abs(mModules[i].getTransVelocity()) - mAvgSpeed);
      }
    }
  }

  /**
   * Sets the current YAW heading as the 0'd heading
   */
  private void resetGyro() {
    mPigeon2.reset(); 
    PoseEstimation.resetGyro();
  }

  /**
   * returns the rate of rotation from the pidgeon in deg/sec CCW positive
   */
  public static double getRate() {
    return -mPigeon2.getRate();
  }

  public static SwerveDriveKinematics getKinematics(){
    return mKinematics;
  }

  /**
   * this gets the Yaw degrees of the gyro in continuous input (360 == 0) CCW (with neg)
   * 
   * @return the degrees at which the gyro is at
   */
  public static double getHeading() {
    return -mPigeon2.getAngle()%360;
  }

  /**
   * This gets the Rotation2d of the gyro (which is in continuous input)
   * 
   * @return the Rotation2d of the gyro CCW POSITIVE(Unit Circle Rise UP)
   * @see Rotation2d
   */
<<<<<<< HEAD
  public static Rotation2d getRotation2d() {
=======
  public Rotation2d getRotation2d() {

    return Rotation2d.fromDegrees(getHeading());
   * [frontleft, frontright, backleft, backright]
   * 
   * @see SwerveModuleState
   * @param desiredStates requires a SwerveModuleState array
   */

<<<<<<< HEAD
  public void setModuleStates(SwerveModuleState[] desiredStates,boolean isOpenLoop) {
    double ts3 = ((double)System.nanoTime())/1000000;
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, mConfig.TELE_MAX_SPEED_MPS);
    for (int i = 0; i < desiredStates.length; i++) {
      mModules[i].setDesiredState(desiredStates[i],isOpenLoop);
    }
    ////System.out.println("Set Module States Nano: " + ((double)System.nanoTime()/1000000-ts3));

=======
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.TELE_MAX_SPEED_MPS);

    for (int i = 0; i < desiredStates.length; i++) {
      mModules[i].setDesiredState(desiredStates[i]);
    }
>>>>>>> main

  }

  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft,
   *         backright]
   * @see SwerveModulePosition
   */
<<<<<<< HEAD
  public static SwerveModulePosition[] getModulePositions() {
=======
  public SwerveModulePosition[] getModulePositions() {
>>>>>>> main
    SwerveModulePosition[] positions = new SwerveModulePosition[mModules.length];

    for (int i = 0; i < mModules.length; i++) {
      positions[i] = mModules[i].getModulePos();
    }

    return positions;

  }

  /**
   * 
   * @param vxMPS this is the forward velocity in meters/second
   * @param vyMPS this is the sideways velocity in meter/second (left is positive)
   * @param angleSpeedRADPS this is in radians/second counterclockwise
   * @param fieldOriented this is a boolean that determines if the robot is field oriented or not
<<<<<<< HEAD
   * @param isOpenLoop this is a boolean that determines if the robot is to use PID+FF for translation and rotation. Highly recommended for auton
=======
>>>>>>> main
   * 
   * @apiNote Keep in mind all of this is field relative so resetting the gyro midmatch will also
   *          reset these params
   */
  public void setChassisSpeed(double vxMPS, double vyMPS, double angleSpeedRADPS,
<<<<<<< HEAD
    boolean fieldOriented, boolean isOpenLoop) {
    double ts1 = (double) System.nanoTime()/1000000;
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      if (!isOpenLoop){
        
        double xPID = chassisXPID.calculate(getChassisSpeeds().vxMetersPerSecond,vxMPS);
        double yPID = chassisYPID.calculate(getChassisSpeeds().vyMetersPerSecond,vyMPS);
        double rotPID = chassisRotPID.calculate(Math.toRadians(getRate()),angleSpeedRADPS);
        
        if(chassisXPID.atSetpoint()){
          xPID = 0;
        }
        if(chassisYPID.atSetpoint()){
          yPID = 0;
        }
        if(chassisRotPID.atSetpoint()){
          rotPID = 0;
        }
       
        SmartDashboard.putNumber("ChassisSpeedXPID", xPID);
        SmartDashboard.putNumber("ChassisSpeedYPID", yPID);
        SmartDashboard.putNumber("ChassisSpeedRotPID", rotPID);

        SmartDashboard.putNumber("ChassisSpeedX", vxMPS);
        SmartDashboard.putNumber("ChassisSpeedy", vyMPS);
        SmartDashboard.putNumber("ChassisSpeedRot", angleSpeedRADPS);

        SmartDashboard.putNumber("ChassisSpeedXError", chassisXPID.getPositionError());
        SmartDashboard.putNumber("ChassisSpeedYError", chassisYPID.getPositionError());
        SmartDashboard.putNumber("ChassisSpeedRotError", chassisRotPID.getPositionError());
        chassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS+xPID,
                                                        vyMPS+yPID, 
                                                        angleSpeedRADPS+rotPID, PoseEstimation.getRobotPose().getRotation());
      }else{
        chassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS, vyMPS, angleSpeedRADPS, PoseEstimation.getRobotPose().getRotation());
      }
      

    } else {
      chassisSpeeds = new ChassisSpeeds(vxMPS, vyMPS, angleSpeedRADPS);
    }
    desiredChassisSpeeds = chassisSpeeds;
    setChassisSpeed(chassisSpeeds,isOpenLoop);

  }

  
  public void setChassisSpeed(ChassisSpeeds chassisSpeeds,boolean isOpenLoop){
    double ts2 = (double) System.nanoTime()/1000000;
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, CompConstants.loopTime);
    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);
    if (CompConstants.DEBUG_MODE){
      mTargetSpeed = moduleStates[0].speedMetersPerSecond;
    }
    //////System.out.println("Set Chassis Speed Time Nano: " + ((double) System.nanoTime()/1000000-ts2));
    setModuleStates(moduleStates,isOpenLoop);
  }


  public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);
    if (CompConstants.DEBUG_MODE){
      mTargetSpeed = moduleStates[0].speedMetersPerSecond;
    }
    setModuleStates(moduleStates,true);
  }


  public static ChassisSpeeds getChassisSpeeds(){
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];
    for (int i = 0; i<4; i++){
      moduleStates[i] = mModules[i].getState();
    }
    return mKinematics.toChassisSpeeds(moduleStates);
  }


  public void setChassisSpeed(double x, double y, double rot,boolean isOpenLoop) {
    setChassisSpeed(x, y, rot,false, isOpenLoop);
  }


  private void resetRobotPose() {
    mOdometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
=======
      boolean fieldOriented) {
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS, vyMPS, angleSpeedRADPS, getRotation2d());
    } else {
      chassisSpeeds = new ChassisSpeeds(vxMPS, vyMPS, angleSpeedRADPS);
    }
    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);


    setModuleStates(moduleStates);

  }

  public void setChassisSpeed(double x, double y, double rot) {
    setChassisSpeed(x, y, rot, false);
  }

  /**
   * This method resets the pose of the robot to the desired robot pose
   * 
   * @param pose provide the new desired pose of the robot
   * @see Pose2d
   */
  public void resetRobotPose(Pose2d pose) {
    mOdometry.resetPosition(pose.getRotation(), getModulePositions(), pose);
>>>>>>> main
  }


  /**
   * @return provide the pose of the robot in meters
   */
<<<<<<< HEAD
  public static Pose2d getRobotPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * @return Raw Modules
   */
  public SwerveModuleAbstract[] getRawModules() {
    return mModules;
  }

=======
  public Pose2d getRobotPose() {
    return mOdometry.getPoseMeters();
  }

>>>>>>> main

}
