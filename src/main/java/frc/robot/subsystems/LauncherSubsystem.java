// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.constants.LauncherConstants.LauncherState;

public class LauncherSubsystem extends SubsystemBase {
  private final CANSparkMax mIndexer = new CANSparkMax(LauncherConstants.INDEXER_ID, MotorType.kBrushless);

  private final TalonFX mLeaderFlywheel = new TalonFX(LauncherConstants.LEADER_FLYWHEEL_ID, "DriveMotors");
  private final TalonFX mFollowerFlywheel = new TalonFX(LauncherConstants.FOLLOWER_FLYWHEEL_ID, "DriveMotors");

  private final DigitalInput mBeamBreak = new DigitalInput(LauncherConstants.BEAMBREAK_ID);

  private double mDesiredVelocity = 0;

  private LauncherMode mLauncherMode = LauncherMode.OFF;

  private boolean mLauncherReady = false;

  public LauncherSubsystem() {
    resetEncoders();
    configureDevices();
  }
  
  public void configureDevices() {
    mLeaderFlywheel.getConfigurator().apply(new TalonFXConfiguration());
    mFollowerFlywheel.getConfigurator().apply(new TalonFXConfiguration());

    mFollowerFlywheel.setControl(new Follower(mLeaderFlywheel.getDeviceID(), false));

    TalonFXConfiguration configuration = new TalonFXConfiguration();
    configuration.Feedback.SensorToMechanismRatio = 1/(LauncherConstants.FLYWHEEL_SENSOR_TO_MECHANISM*60);

    Slot0Configs slotConfigs = new Slot0Configs();
    slotConfigs.kP = LauncherConstants.LAUNCHER_PID_CONTROLLER.getP();
    slotConfigs.kI = LauncherConstants.LAUNCHER_PID_CONTROLLER.getI();
    slotConfigs.kD = LauncherConstants.LAUNCHER_PID_CONTROLLER.getD();
    slotConfigs.kS = LauncherConstants.LAUNCHER_FF_CONTROLLER.ks;
    slotConfigs.kV = LauncherConstants.LAUNCHER_FF_CONTROLLER.kv;

    mLeaderFlywheel.getConfigurator().apply(configuration.withSlot0(slotConfigs));
    mFollowerFlywheel.getConfigurator().apply(configuration.withSlot0(slotConfigs));

    mLeaderFlywheel.setNeutralMode(NeutralModeValue.Coast);
    mFollowerFlywheel.setNeutralMode(NeutralModeValue.Coast);



    mIndexer.restoreFactoryDefaults();
    mIndexer.setInverted(LauncherConstants.INDEXER_IS_INVERTED);
    mIndexer.setIdleMode(IdleMode.kBrake);
    mIndexer.burnFlash();
  }

  @Override
  public void periodic() {
    double actualVelocity = mLeaderFlywheel.getVelocity().getValueAsDouble();



    mLauncherReady = Math.abs(Math.abs(mDesiredVelocity) - Math.abs(actualVelocity)) <= LauncherConstants.FLYWHEEL_TOLERANCE && 
                     mDesiredVelocity != 0 && mLauncherMode != LauncherMode.PASSIVE;
    runLauncher();

    SmartDashboard.putBoolean("Piece in Indexer", pieceInIndexer());
    SmartDashboard.putNumber("Flywheel Speed", actualVelocity);
    SmartDashboard.putNumber("Indexer Applied Output", mIndexer.getAppliedOutput());
  }

  public void runIndexer(double velocity) {
    mIndexer.set(velocity);
  }
  
  public void runLauncher() {
    //predicted velocity values
    if (mLauncherMode == LauncherMode.SPEAKER) {
      mDesiredVelocity = LauncherState.SPEAKER_DEFAULT.velocity;
    }
    else if (mLauncherMode == LauncherMode.AMP) {
      mDesiredVelocity = LauncherState.AMP_DEFAULT.velocity;
    }
    else if (mLauncherMode == LauncherMode.TRAP) {
      mDesiredVelocity = LauncherState.TRAP_DEFAULT.velocity;
    }
    else if (mLauncherMode == LauncherMode.PASSIVE) {
      mDesiredVelocity = LauncherState.PASSIVE_DEFAULT.velocity;
    }
    else if (mLauncherMode == LauncherMode.OFF) {
      mDesiredVelocity = 0;
    }

    mLeaderFlywheel.setControl(new VoltageOut(mDesiredVelocity*12/LauncherConstants.FLYWHEEL_MAX_VELOCITY));
    // mLeaderFlywheel.setControl(new VelocityVoltage(mDesiredVelocity).withSlot(0));

  }

  public boolean isIndexerOn() {
    return Math.abs(mIndexer.getAppliedOutput()) > 0;
  }

  public boolean pieceInIndexer(){
    return !mBeamBreak.get();
  }

  public boolean isLauncherReady() {
    return true; // Returning True until we set up velocity PID
  }

  public void setLauncherMode(LauncherMode mode) {
    mLauncherMode = mode;
  }
  
  public void stopLauncher() {
    mLauncherMode = LauncherMode.OFF;
  }

  private void resetEncoders() {
    mLeaderFlywheel.setPosition(0);
    mFollowerFlywheel.setPosition(0);
  }

  public Command waitUntilFlywheelSetpointCommand() {
    return new FunctionalCommand(() -> setLauncherMode(mLauncherMode), null, null, this::isLauncherReady, this);    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double[] getLauncherSpeeds() {
    return new double[] {mIndexer.get(), mLeaderFlywheel.get(), mFollowerFlywheel.get()};
  }
}
