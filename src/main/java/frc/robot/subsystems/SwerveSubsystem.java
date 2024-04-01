// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.constants.CompConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.robots.SwerveConfig;
import frc.robot.swerve.SwerveModuleAbstract;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private static SwerveDriveKinematics mKinematics;
  private static SwerveDrivePoseEstimator mOdometry;

  private static Pigeon2 mPigeon2;

  private static SwerveModuleAbstract[] mModules;
  private double mTargetSpeed = 0;
  private double mAvgSpeed = 0;
  private double maxSpeed = 0;
  private PIDController chassisRotPID =  new PIDController(0, 0, 0);
  private PIDController chassisXPID = new PIDController(0, 0, 0);
  private PIDController chassisYPID = new PIDController(0, 0, 0);
  public final SwerveConfig mConfig;
  private ChassisSpeeds desiredChassisSpeeds = new ChassisSpeeds();
  private Field2d mField = new Field2d();
  private SwerveDriveOdometry jank;


  public SwerveSubsystem(SwerveConfig configuration) {
    // Populating Instance Variables

    chassisRotPID.setTolerance(0.2);
    chassisXPID.setTolerance(0.2);
    chassisYPID.setTolerance(0.2);
    
    mConfig = configuration;
    mKinematics = mConfig.SWERVE_KINEMATICS;
    mPigeon2 = mConfig.PIGEON;
    mModules = mConfig.SWERVE_MODULES;
    mOdometry = new SwerveDrivePoseEstimator(mKinematics, getRotation2d(), getModulePositions(), new Pose2d());
    jank = new SwerveDriveOdometry(mKinematics, getRotation2d(), getModulePositions());

    resetGyro();
    resetRobotPose();


    SmartDashboard.putData("Field", mField);
  }

  @Override
  public void periodic() {
    //jank.update(getRotation2d(), getModulePositions());
    // This method will be called once per scheduler run    
    mOdometry.update(getRotation2d(), getModulePositions());

    SmartDashboard.putBoolean("SpeakerAligned", DefaultDriveCommand.getAlignedToSpeaker());

    //SmartDashboard.putNumber("Pose Est Error", jank.getPoseMeters().minus(mOdometry.getEstimatedPosition()).getTranslation().getNorm());
    SmartDashboard.putNumber("XPos", mOdometry.getEstimatedPosition().getX());
    SmartDashboard.putNumber("YPos", mOdometry.getEstimatedPosition().getY());
    SmartDashboard.putNumber("Rot", mOdometry.getEstimatedPosition().getRotation().getDegrees());
    SmartDashboard.putNumber("Heading", getRotation2d().getDegrees());
    SmartDashboard.putNumber("DistToSpeaker",FieldConstants.getSpeakerDistance(getRobotPose()));

    mField.setRobotPose(mOdometry.getEstimatedPosition());
    double current = 0;
    for (int i = 0; i<4; i++) {
      current += mModules[i].getCurrent();
    }
    SmartDashboard.putNumber("Swerve Current", current);

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
  public void resetGyro() {
    mPigeon2.setYaw(0); 
    mOdometry.resetPosition(new Rotation2d(0), getModulePositions(), getRobotPose());
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
  public static Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
  }

  /**
   * This function sets the current speeds of the swerve modules to the following array pattern
   * [frontleft, frontright, backleft, backright]
   * 
   * @see SwerveModuleState
   * @param desiredStates requires a SwerveModuleState array
   */

  public void setModuleStates(SwerveModuleState[] desiredStates,boolean isOpenLoop) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, mConfig.TELE_MAX_SPEED_MPS);
    for (int i = 0; i < desiredStates.length; i++) {
      mModules[i].setDesiredState(desiredStates[i],isOpenLoop);
    }
  }

  /**
   * 
   * @return an array of SwerveModulePosition objects as [frontleft, frontright, backleft,
   *         backright]
   * @see SwerveModulePosition
   */
  public static SwerveModulePosition[] getModulePositions() {
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
   * @param isOpenLoop this is a boolean that determines if the robot is to use PID+FF for translation and rotation. Highly recommended for auton
   * 
   * @apiNote Keep in mind all of this is field relative so resetting the gyro midmatch will also
   *          reset these params
   */
  public void setChassisSpeed(double vxMPS, double vyMPS, double angleSpeedRADPS,
    boolean fieldOriented, boolean isOpenLoop) {
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
       
        // SmartDashboard.putNumber("ChassisSpeedXPID", xPID);
        // SmartDashboard.putNumber("ChassisSpeedYPID", yPID);
        // SmartDashboard.putNumber("ChassisSpeedRotPID", rotPID);

        // SmartDashboard.putNumber("ChassisSpeedX", vxMPS);
        // SmartDashboard.putNumber("ChassisSpeedy", vyMPS);
        // SmartDashboard.putNumber("ChassisSpeedRot", angleSpeedRADPS);

        // SmartDashboard.putNumber("ChassisSpeedXError", chassisXPID.getPositionError());
        // SmartDashboard.putNumber("ChassisSpeedYError", chassisYPID.getPositionError());
        // SmartDashboard.putNumber("ChassisSpeedRotError", chassisRotPID.getPositionError());
        chassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS+xPID,
                                                        vyMPS+yPID, 
                                                        angleSpeedRADPS+rotPID, getRotation2d());
      }else{
        chassisSpeeds =  ChassisSpeeds.fromFieldRelativeSpeeds(vxMPS, vyMPS, angleSpeedRADPS, getRotation2d());
      }
      

    } else {
      chassisSpeeds = new ChassisSpeeds(vxMPS, vyMPS, angleSpeedRADPS);
    }
    desiredChassisSpeeds = chassisSpeeds;
    setChassisSpeed(chassisSpeeds,isOpenLoop);

  }

  
  public void setChassisSpeed(ChassisSpeeds chassisSpeeds,boolean isOpenLoop){
    chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, CompConstants.LOOP_TIME);
    SwerveModuleState[] moduleStates = mKinematics.toSwerveModuleStates(chassisSpeeds);
    if (CompConstants.DEBUG_MODE){
      mTargetSpeed = moduleStates[0].speedMetersPerSecond;
    }
    setModuleStates(moduleStates,isOpenLoop);
  }


  public void setChassisSpeed(ChassisSpeeds chassisSpeeds){
    setChassisSpeed(chassisSpeeds, false);
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

  public void resetRobotPose() {
    mOdometry.resetPosition(getRotation2d(), getModulePositions(), new Pose2d());
  }

  public void resetRobotPose(Pose2d pose) {
    mOdometry.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public static void updateVision(EstimatedRobotPose pose) {
    mOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds);
  }
  public static void updateVision(EstimatedRobotPose pose, Matrix<N3, N1> std) {
    mOdometry.addVisionMeasurement(pose.estimatedPose.toPose2d(), pose.timestampSeconds, std);
  }
  /**
   * @return provide the pose of the robot in meters
   */
  public static Pose2d getRobotPose() {
    return mOdometry.getEstimatedPosition();
  }

  /**
   * @return Raw Modules
   */
  public SwerveModuleAbstract[] getRawModules() {
    return mModules;
  }

  public static double getRoll(){
    return mPigeon2.getRoll().getValueAsDouble();
  }


}
