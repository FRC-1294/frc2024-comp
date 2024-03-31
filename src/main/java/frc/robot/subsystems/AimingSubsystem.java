// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.lang.reflect.Field;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import javax.swing.border.LineBorder;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.CompConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.commands.DefaultMechCommand;
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

  //0.1 is the non-zero sample time and 0.02 is our loop time 
  LinearFilter mAbsEncFilter = LinearFilter.singlePoleIIR(0.1,0.02);

  public AimingSubsystem() {
    mChooser.addOption("Brake", AimingMotorMode.BRAKE);
    mChooser.addOption("Coast", AimingMotorMode.COAST);
    mChooser.setDefaultOption("Brake", AimingConstants.INITIAL_MOTOR_MODE);
    SmartDashboard.putData("Elevator Motor Mode", mChooser);

    mLeftElevatorEncoder = mLeftElevatorMotor.getEncoder();
    mRightElevatorEncoder = mRightElevatorMotor.getEncoder();
    configureDevices();
    BooleanSupplier getBrake = ()-> mChooser.getSelected() == AimingMotorMode.BRAKE;
    BooleanSupplier getCoast = ()-> mChooser.getSelected() == AimingMotorMode.COAST;

    new Trigger(getBrake).onTrue(
      new InstantCommand(()->{
    mLeftElevatorMotor.setIdleMode(IdleMode.kBrake);
    mRightElevatorMotor.setIdleMode(IdleMode.kBrake);

    mRightWristMotor.setIdleMode(IdleMode.kBrake);
    mLeftWristMotor.setIdleMode(IdleMode.kBrake);}).ignoringDisable(true));

    new Trigger(getCoast).onTrue(
      new InstantCommand(()->{
    mLeftElevatorMotor.setIdleMode(IdleMode.kCoast);
    mRightElevatorMotor.setIdleMode(IdleMode.kCoast);

    mRightWristMotor.setIdleMode(IdleMode.kCoast);
    mLeftWristMotor.setIdleMode(IdleMode.kCoast);}).ignoringDisable(true));
    mWristController.setIntegratorRange(-0.1, 0.1);
    mWristController.setIZone(3);
  }

  // Setting Conversions and Inversions
  public void configureDevices() {

    mLeftElevatorMotor.restoreFactoryDefaults();
    mRightElevatorMotor.restoreFactoryDefaults();
    mLeftWristMotor.restoreFactoryDefaults();
    mRightWristMotor.restoreFactoryDefaults();

    //initialize PID Controller Constants for SlotConfigs    
    mWristController = AimingConstants.mWristPIDConstants.toWPIController();
    //note: configuration uses internal encoders inside the motors, subject to change
  
    mLeftElevatorMotor.setInverted(!AimingConstants.ELEVATOR_LEFT_IS_NORMAL);
    mLeftWristMotor.setInverted(!AimingConstants.WRIST_LEFT_IS_NORMAL);
    mWristController.setTolerance(AimingConstants.WRIST_TOLERANCE_DEG);
    mElevatorController.setTolerance(AimingConstants.ELEVATOR_TOLERANCE_IN);

    // mLeftElevatorMotor.setSmartCurrentLimit(70);
    // mRightElevatorMotor.setSmartCurrentLimit(10);

    mLeftWristMotor.setSmartCurrentLimit(100);
    mRightWristMotor.setSmartCurrentLimit(100);

    //follower configuration
    mRightWristMotor.follow(mLeftWristMotor, true);
    mRightElevatorMotor.follow(mLeftElevatorMotor, false);

    // mLeftElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);
    // mRightElevatorEncoder.setPositionConversionFactor(AimingConstants.ELEVATOR_ROTATIONS_TO_METERS);

    mLeftElevatorEncoder.setPositionConversionFactor(1.0);
    mRightElevatorEncoder.setPositionConversionFactor(1.0);

    mLeftElevatorEncoder.setPosition(0);
    mRightElevatorEncoder.setPosition(0);

    // Don't even ask
    mWristThroughBoreEncoder.setConnectedFrequencyThreshold(AimingConstants.CONNECTION_THRESH_HZ);

    mLeftElevatorMotor.burnFlash();
    mRightElevatorMotor.burnFlash();
    mLeftWristMotor.burnFlash();
    mRightWristMotor.burnFlash();

    SmartDashboard.putNumber("wristOUtput", 0);

  }

  @Override
  public void periodic() {
    mCurrentWristRotationDeg = getCurrentWristDegreees();
    mCurrentElevatorDistanceIn = getCurrentElevatorDistance();
    SmartDashboard.putNumber("Current Wrist Rotation", getCurrentWristDegreees());
    SmartDashboard.putNumber("Current Elevator Extension", getCurrentElevatorDistance());
    SmartDashboard.putNumber("Desired Elevator Extension", getDesiredElevatorDistance());
    SmartDashboard.putNumber("Desired Wrist Rotation", getDesiredWristRotation());
    updateMotorModes();
    // elevatorPeriodic();
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
    
    // Reducing Max PID Thingy if Wrist  Down and Stuff Because Breaky
    double maxPIDContribution = AimingConstants.MAX_WRIST_PID_CONTRIBUTION;

    if (wristPIDCalculation < 0) {
      maxPIDContribution = 0.3;
    }
    if (wristPIDCalculation < 0 && mCurrentWristRotationDeg<20) {
      maxPIDContribution *= mCurrentWristRotationDeg/20;
    } 

    wristPIDCalculation = MathUtil.clamp(wristPIDCalculation, -maxPIDContribution, maxPIDContribution);

    double wristFeedforwardCalculation = Math.cos(Math.toRadians(mCurrentWristRotationDeg-AimingConstants.COG_OFFSET))*AimingConstants.WRIST_KG;
    mLeftWristMotor.set(wristPIDCalculation + wristFeedforwardCalculation + Math.signum(wristPIDCalculation)*AimingConstants.WRIST_KS);

  }

  private void updateMotorModes() {
    SmartDashboard.updateValues();
  }

  // Contains Smart Dashboard Statements ONLY ON DEBUG
  private void debugSmartDashboard() {
      SmartDashboard.putNumber("Desired Wrist Rotation", getDesiredWristRotation());

    if (CompConstants.DEBUG_MODE || CompConstants.PID_TUNE_MODE) {
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
      SmartDashboard.putNumber("Current Launch Angle", getCurrentLaunchDegrees());
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
    double raw_deg = -(mWristThroughBoreEncoder.getAbsolutePosition()*AimingConstants.WRIST_THROUGHBORE_GEAR_RATIO*360 - AimingConstants.WRIST_THROUGHBORE_ENCODER_OFFSET);
    return raw_deg;
  }

  public double getCurrentLaunchDegrees(){
    return -mCurrentWristRotationDeg + AimingConstants.REST_LAUNCH_ANGLE;
  }

  public AimState getCurrentState(){
    for (AimState state : AimState.values()){
      if (state.withinWristTolerance(getCurrentWristDegreees())
        && state.withinElevatorTolerance(getCurrentElevatorDistance()) && state != AimState.AUTO_AIM){
        return state;
      }
    }
    return AimState.AUTO_AIM;
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

  public void setDesiredWristRotation(Supplier<Double> sp) {
    mDesiredWristRotationDeg = sp.get();
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
    if(state != AimState.AUTO_AIM){
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

  public void setWristToleranceDeg(Supplier<Double> tolerance){
    mWristController.setTolerance(tolerance.get());
  }
  
  public void setElevatorToleranceDeg(double tolerance){
    mElevatorController.setTolerance(tolerance);
  }

  public boolean atElevatorSetpoint() {
    return mElevatorController.atSetpoint();
  }
  public boolean atWristSetpoint() {
    return mWristController.atSetpoint();
  }

  public boolean atSetpoints() {
    return atWristSetpoint();
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

  public Command waitUntilWristSetpoint(Supplier<Double> sp, Supplier<Double> tolerance) {
    return new FunctionalCommand(() -> {setDesiredWristRotation(sp);
    setWristToleranceDeg(tolerance);},
    ()->{}, (Interruptable)->{}, this::atWristSetpoint, this);
  }

  public Command waitUntilAutoAimSetpointTracked() {
    return new FunctionalCommand(()-> {},
                                 () -> {setDesiredWristRotation(() -> AimingConstants.getPolynomialRegression());
                                        setWristToleranceDeg(()->AimingConstants.getAutoAimWristToleranceDegrees());},
                                (Interruptable)->{},
                                ()->DefaultMechCommand.mDesiredState != AimState.AUTO_AIM, 
                                this
                                );
  }

    public Command waitUntilCalculationAutoAimSetpointTracked() {
      return new FunctionalCommand(()-> {},
                                  () -> {setDesiredLaunchRotation(Math.toDegrees(Autoaim.getNeededLaunchAngle()));
                                          setWristToleranceDeg(()->AimingConstants.getAutoAimWristToleranceDegrees());},
                                  (Interruptable)->{},
                                  ()->DefaultMechCommand.mDesiredState != AimState.AUTO_AIM, 
                                  this
                                  );
    } 

  public Command waitUntilAutoAimSetpoint() {
    return new FunctionalCommand(()-> {},
                                 () -> {setDesiredWristRotation(() -> AimingConstants.getPolynomialRegression());
                                        setWristToleranceDeg(()->AimingConstants.getAutoAimWristToleranceDegrees());},
                                (Interruptable)->{},
                                this::atWristSetpoint, 
                                this
                                );
  }
}