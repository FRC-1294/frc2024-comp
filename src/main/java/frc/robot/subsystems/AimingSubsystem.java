// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.CompConstants;
import frc.robot.constants.AimState;


public class AimingSubsystem extends SubsystemBase {
  public enum AimingMotorMode {COAST, BRAKE}

  // Elevator Hardware
  private final CANSparkMax mLeftElevatorMotor = new CANSparkMax(AimingConstants.LEFT_ELEVATOR_SPARK_ID, MotorType.kBrushless);
  private final CANSparkMax mRightElevatorMotor = new CANSparkMax(AimingConstants.RIGHT_ELEVATOR_SPARK_ID, MotorType.kBrushless);
  
  // Wrist Hardware
  private final CANSparkMax mLeftWristMotor = new CANSparkMax(AimingConstants.LEFT_WRIST_SPARK_ID, MotorType.kBrushless);
  private final CANSparkMax mRightWristMotor = new CANSparkMax(AimingConstants.RIGHT_WRIST_SPARK_ID, MotorType.kBrushless);
  private final DutyCycleEncoder mWristThroughBoreEncoder = new DutyCycleEncoder(AimingConstants.WRIST_THROUGHBORE_ENCODER_ID);

  // Current States
  private double mCurrentElevatorDistanceIn = AimingConstants.MIN_ELEVATOR_DIST_METERS;
  private double mCurrentWristRotationDeg = AimingConstants.MIN_WRIST_ROTATION_DEG;

  // Desired States
  private double mDesiredElevatorDistanceIn = mCurrentElevatorDistanceIn;
  private double mDesiredWristRotationDeg = mCurrentWristRotationDeg;


  // MotorMode Chooser
  private final SendableChooser<AimingMotorMode> mChooser = new SendableChooser<>();

  // PID Controllers and Motor Configs 
  private PIDController mElevatorController = AimingConstants.mElevatorPIDConstants.toWPIController();  
  private PIDController mWristController = AimingConstants.mWristPIDConstants.toWPIController();
  private RelativeEncoder mLeftElevatorEncoder;
  private RelativeEncoder mRightElevatorEncoder;

  ArmFeedforward mWristFeedforwardController = new ArmFeedforward(AimingConstants.mWristPIDConstants.mKS, AimingConstants.WRIST_KG, AimingConstants.mWristPIDConstants.mKV);

  MotorOutputConfigs mLeftWristMotorOutputConfigs = new MotorOutputConfigs();
  MotorOutputConfigs mRightWristMotorOutputConfigs = new MotorOutputConfigs();

  Slot0Configs mElevatorControllerSlot0Configs = new Slot0Configs();

  public AimingSubsystem() {
    mChooser.addOption("Brake", AimingMotorMode.BRAKE);
    mChooser.addOption("Coast", AimingMotorMode.COAST);
    mChooser.setDefaultOption("Brake", AimingConstants.INITIAL_MOTOR_MODE);
    SmartDashboard.putData("Elevator Motor Mode", mChooser);

    mLeftElevatorEncoder = mLeftElevatorMotor.getEncoder();
    mRightElevatorEncoder = mRightElevatorMotor.getEncoder();
    configureDevices();
  }

  // Setting Conversions and Inversions
  public void configureDevices() {

    mLeftElevatorMotor.restoreFactoryDefaults();
    mRightElevatorMotor.restoreFactoryDefaults();

    //initialize PID Controller Constants for SlotConfigs    
    mWristController = AimingConstants.mWristPIDConstants.toWPIController();
    //note: configuration uses internal encoders inside the motors, subject to change
  
    mLeftElevatorMotor.setInverted(!AimingConstants.ELEVATOR_LEFT_IS_NORMAL);
    mLeftWristMotor.setInverted(!AimingConstants.WRIST_LEFT_IS_NORMAL);
    mWristController.setTolerance(AimingConstants.WRIST_TOLERANCE_DEG);
    mElevatorController.setTolerance(AimingConstants.ELEVATOR_TOLERANCE_IN);

    mLeftElevatorMotor.setSmartCurrentLimit(40);
    mRightElevatorMotor.setSmartCurrentLimit(40);

    mLeftWristMotor.setSmartCurrentLimit(80);
    mRightWristMotor.setSmartCurrentLimit(80);

    //follower configuration
    mRightWristMotor.follow(mLeftWristMotor, true);
    mRightElevatorMotor.follow(mLeftElevatorMotor, true);

    mLeftElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);
    mRightElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);

    mLeftElevatorEncoder.setPosition(0);
    mRightElevatorEncoder.setPosition(0);

    // Don't even ask
    mWristThroughBoreEncoder.setConnectedFrequencyThreshold(AimingConstants.CONNECTION_THRESH_HZ);

    mLeftElevatorMotor.burnFlash();
    mRightElevatorMotor.burnFlash();
    mLeftWristMotor.burnFlash();
    mRightWristMotor.burnFlash();
  }

  @Override
  public void periodic() {
    mCurrentWristRotationDeg = getCurrentWristDegreees();
    mCurrentElevatorDistanceIn = getCurrentElevatorDistance();

    updateMotorModes();
    elevatorPeriodic();
    wristPeriodic();
    debugSmartDashboard();
  }

  private void elevatorPeriodic() {
    //Clamping Rotation between domain
    mDesiredElevatorDistanceIn = MathUtil.clamp(mDesiredElevatorDistanceIn, AimingConstants.MIN_ELEVATOR_DIST_METERS, AimingConstants.MAX_ELEVATOR_DIST_METERS);

    //Temp Regular PID
    double elevatorPIDCalculation = mElevatorController.calculate(mCurrentElevatorDistanceIn, mDesiredElevatorDistanceIn);
    elevatorPIDCalculation = MathUtil.clamp(elevatorPIDCalculation, -AimingConstants.MAX_ELEVATOR_PID_CONTRIBUTION, AimingConstants.MAX_ELEVATOR_PID_CONTRIBUTION);

    mLeftElevatorMotor.set(elevatorPIDCalculation);
  }


  private void wristPeriodic() {
    //Clamping Rotation between domain
    mDesiredWristRotationDeg = MathUtil.clamp(mDesiredWristRotationDeg, AimingConstants.MIN_WRIST_ROTATION_DEG, AimingConstants.MAX_WRIST_ROTATION);

    double wristPIDCalculation = mWristController.calculate(mCurrentWristRotationDeg, mDesiredWristRotationDeg);  
    wristPIDCalculation = MathUtil.clamp(wristPIDCalculation, -AimingConstants.MAX_WRIST_PID_CONTRIBUTION, AimingConstants.MAX_WRIST_PID_CONTRIBUTION);
    

    double wristFeedforwardCalculation = Math.cos(Math.toRadians(mCurrentWristRotationDeg-AimingConstants.COG_OFFSET))*AimingConstants.WRIST_KG;
    mLeftWristMotor.set(wristPIDCalculation + wristFeedforwardCalculation);
  }

  private void updateMotorModes() {
    SmartDashboard.updateValues();

    AimingMotorMode mode = mChooser.getSelected();
    
    // Motors go towards setpoints
    IdleMode idleMode;

    switch (mode) {
        case COAST:
            idleMode = IdleMode.kCoast;
            break;
        default:
            idleMode = IdleMode.kBrake;
            break;
    }

    mLeftElevatorMotor.setIdleMode(idleMode);
    mRightElevatorMotor.setIdleMode(idleMode);

    mRightWristMotor.setIdleMode(idleMode);
    mLeftWristMotor.setIdleMode(idleMode);
  }

  // Contains Smart Dashboard Statements ONLY ON DEBUG
  private void debugSmartDashboard() {
    if (true || CompConstants.DEBUG_MODE || CompConstants.PID_TUNE_MODE) {
      SmartDashboard.putNumber("Current Wrist Rotation", getCurrentWristDegreees());
      SmartDashboard.putNumber("Current Elevator Distance", getCurrentElevatorDistance());
      SmartDashboard.putNumber("Desired Wrist Rotation", getDesiredWristRotation());
      SmartDashboard.putNumber("Desired Elevator Distance", getDesiredElevatorDistance());
      SmartDashboard.putNumber("Raw Elevator Encoder Position", mLeftElevatorEncoder.getPosition());
      SmartDashboard.putNumber("Raw Wrist Encoder Rotation", mWristThroughBoreEncoder.getAbsolutePosition());
      SmartDashboard.putBoolean("At Elevator setpoint", atElevatorSetpoint());
      SmartDashboard.putBoolean("At Wrist setpoint", atWristSetpoint());
      SmartDashboard.putNumber("Throughbore Encoder Position", mWristThroughBoreEncoder.getAbsolutePosition()*AimingConstants.WRIST_THROUGHBORE_GEAR_RATIO*360 - AimingConstants.WRIST_THROUGHBORE_ENCODER_OFFSET);
      SmartDashboard.putBoolean("Wrist Throughbore Is Connected", mWristThroughBoreEncoder.isConnected());
      SmartDashboard.putNumber("Wrist Throughbore Frequency", mWristThroughBoreEncoder.getFrequency());
      SmartDashboard.putNumber("Wrist Tolerance", mWristController.getPositionTolerance());
      SmartDashboard.putNumber("Wrist Error", mWristController.getPositionError());
    }

    if (CompConstants.PID_TUNE_MODE) {
      SmartDashboard.putNumber("Elevator P", AimingConstants.mElevatorPIDConstants.mKP);
      SmartDashboard.putNumber("Elevator I", AimingConstants.mElevatorPIDConstants.mKI);
      SmartDashboard.putNumber("Elevator D", AimingConstants.mElevatorPIDConstants.mKD);
      SmartDashboard.putNumber("Wrist P", AimingConstants.mWristPIDConstants.mKP);
      SmartDashboard.putNumber("Wrist I", AimingConstants.mWristPIDConstants.mKI);
      SmartDashboard.putNumber("Wrist D", AimingConstants.mWristPIDConstants.mKD);


      double elevatorP = SmartDashboard.getNumber("Elevator P", AimingConstants.mElevatorPIDConstants.mKP);
      double elevatorI = SmartDashboard.getNumber("Elevator I", AimingConstants.mElevatorPIDConstants.mKI);
      double elevatorD = SmartDashboard.getNumber("Elevator D", AimingConstants.mElevatorPIDConstants.mKD);
      double wristP = SmartDashboard.getNumber("Wrist P", AimingConstants.mWristPIDConstants.mKP);
      double wristI = SmartDashboard.getNumber("Wrist I", AimingConstants.mWristPIDConstants.mKI);
      double wristD = SmartDashboard.getNumber("Wrist D", AimingConstants.mWristPIDConstants.mKD);

      mElevatorController = new PIDController(elevatorP, elevatorI, elevatorD);
      mWristController = new PIDController(wristP, wristI, wristD);

      mElevatorController.setTolerance(AimingConstants.ELEVATOR_TOLERANCE_IN);
      mWristController.setTolerance(AimingConstants.WRIST_TOLERANCE_DEG);

    }

    
  }

  // Getters and Setters of States
  public double getCurrentElevatorDistance() {
    return mLeftElevatorEncoder.getPosition();
  }

  /**
   * 
   * @return Current Wrist Rotation in Degrees
   */
  public double getCurrentWristDegreees(){
    return -(mWristThroughBoreEncoder.getAbsolutePosition()*AimingConstants.WRIST_THROUGHBORE_GEAR_RATIO*360 - AimingConstants.WRIST_THROUGHBORE_ENCODER_OFFSET);
  }

  public double getCurrentLaunchDegrees(){
    return -mCurrentWristRotationDeg + AimingConstants.REST_LAUNCH_ANGLE;
  }

  public AimState getCurrentState(){
    for (AimState state : AimState.values()){
      if (state.withinWristTolerance(getCurrentWristDegreees())
        && state.withinElevatorTolerance(getCurrentElevatorDistance()) && state != AimState.TRANSITION){
        return state;
      }
    }
    return AimState.TRANSITION;
  }

  public double getDesiredElevatorDistance() {
    return mDesiredElevatorDistanceIn;
  }
  
  public double getDesiredWristRotation() {
    return mDesiredWristRotationDeg;
  }

  public double getDesiredLaunchRotation() {
    return -mDesiredWristRotationDeg + AimingConstants.REST_LAUNCH_ANGLE;
  }

  public void setDesiredElevatorDistance(double distance) {
    mDesiredElevatorDistanceIn = distance;
  }
  public void setDesiredElevatorDistance(double distance, double tolerance) {
    mDesiredElevatorDistanceIn = distance;
    mElevatorController.setTolerance(tolerance);
  }

  public void setDesiredLaunchRotation(double launchDegrees){
    mDesiredWristRotationDeg = -launchDegrees + AimingConstants.REST_LAUNCH_ANGLE;
  }

  public void setDesiredWristRotation(double rotation) {
    mDesiredWristRotationDeg = rotation;
  }

  public void setDesiredWristRotation(double rotation, double tolerance) {
    mWristController.setTolerance(tolerance);
    mDesiredWristRotationDeg = rotation;
  }

  public void setDesiredSetpoint(AimState state) {
    if(state != AimState.TRANSITION){
      mDesiredElevatorDistanceIn = state.mElevatorHeightMeters;
      mDesiredWristRotationDeg = state.mWristAngleDegrees;
      mWristController.setTolerance(state.mWristToleranceDegrees);
      mElevatorController.setTolerance(state.mElevatorToleranceMeters);
    }
  }

  public void changeDesiredElevatorPosition(double increment) {
    mDesiredElevatorDistanceIn += increment;
  }

  public void changeDesiredWristRotation(double increment) {
    mDesiredWristRotationDeg += increment;
  }

  public void setWristToleranceDeg(double tolerance){
    mWristController.setTolerance(tolerance);
  }
  
  public void setElevatorToleranceDeg(double tolerance){
    mElevatorController.setTolerance(tolerance);
  }

  public void calculateExitAngle(){
    //Ankit do ur thing
  }

  public boolean atElevatorSetpoint() {
    return mElevatorController.atSetpoint();
  }
  public boolean atWristSetpoint() {
    return mWristController.atSetpoint();
  }

  public boolean atSetpoints() {
    return atElevatorSetpoint() && atWristSetpoint();
    
  }

  public Command waitUntilSetpoint(AimState state) {
    return new FunctionalCommand(() -> setDesiredSetpoint(state), ()->{}, (Interruptable)->{}, this::atSetpoints, this);  
  }

  public Command waitUntilWristSetpoint(double wristSP, double wristTolerance) {
    return new FunctionalCommand(()->setDesiredWristRotation(wristSP, wristTolerance), ()->{}, (Interruptable)->{}, this::atWristSetpoint, this);
  }

  public Command waitUntilElevatorSetpoint(double sp) {
    return new FunctionalCommand(() -> setDesiredElevatorDistance(sp), ()->{}, (Interruptable)->{}, this::atElevatorSetpoint, this);  
  }

  public Command waitUntilWristSetpoint(double sp) {
    return new FunctionalCommand(() -> setDesiredWristRotation(sp), ()->{}, (Interruptable)->{}, this::atWristSetpoint, this);  
  }
}