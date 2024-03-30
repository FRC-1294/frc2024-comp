// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.JoystickConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.states.mech_states.Intaken;
import frc.robot.states.mech_states.ReadyForAim;
import frc.robot.states.mech_states.ReadyForHandoff;
import frc.robot.states.mech_states.ReadyForIntake;
import frc.robot.states.mech_states.ReadyForLaunch;
import frc.robot.states.mech_states.UltraInstinct;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.Input;
import frc.robot.subsystems.LauncherSubsystem;

public class DefaultMechCommand{
    private static IntakeSubsystem mIntakeSubsystem;
    private static LauncherSubsystem mLauncherSubsystem;
    private static AimingSubsystem mAimingSubsystem;

    private static MechState mReadyForIntake;
    private static MechState mIntaken;
    private static MechState mReadyForHandoff;
    private static MechState mReadyForAim;
    private static MechState mReadyForLaunch;
    private static MechState mUltraInstinct;
 

    private static boolean mUseUltraInstinct = false;
    private static MechState mMechState;
    public static AimState mDesiredState = AimState.HANDOFF; 

    public DefaultMechCommand(IntakeSubsystem intakeSubsystem, LauncherSubsystem launcherSubsystem, AimingSubsystem aimingSubsystem) {
        mIntakeSubsystem = intakeSubsystem;
        mLauncherSubsystem = launcherSubsystem;
        mAimingSubsystem = aimingSubsystem;

        //addRequirements(mIntakeSubsystem, mLauncherSubsystem, mAimingSubsystem);

        mReadyForIntake = new ReadyForIntake(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mIntaken = new Intaken(launcherSubsystem, aimingSubsystem, intakeSubsystem);
        mReadyForHandoff = new ReadyForHandoff(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mReadyForAim = new ReadyForAim(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mReadyForLaunch = new ReadyForLaunch(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        mUltraInstinct = new UltraInstinct(mLauncherSubsystem,mAimingSubsystem,mIntakeSubsystem);
        
        mMechState = mUltraInstinct;
        mMechState = determineState();

        defineEventListeners();

    }

    public static void defineEventListeners(){
        SmartDashboard.putNumber("Set Wrist Encoder Degrees", 0);
        //Brakes the indexer ONE time when the left trigger is released
        BooleanSupplier getIndexer = ()-> Math.abs(Input.getLeftTrigger()) < LauncherConstants.INDEX_TRIGGER_DEADZONE;
        new Trigger(getIndexer).onTrue(mMechState.brakeIndexer());
        
        //Brakes the indexer ONE time when the current state transitions to ready to launch or ready to aim
        BooleanSupplier returnToIndex = ()-> mMechState == mReadyForAim || determineState() == mReadyForLaunch;
        new Trigger(returnToIndex).onTrue(mMechState.brakeIndexer());

        BooleanSupplier intaken = ()-> mMechState == mIntaken;
        new Trigger(intaken).onTrue(new InstantCommand(()->{
            Input.enableRumble(JoystickConstants.XBOX_RUMBLE_SOFT);
        }));

        BooleanSupplier readyToAim = ()-> mMechState == mReadyForAim;
        new Trigger(readyToAim).onTrue(new InstantCommand(()->Input.enableRumble(JoystickConstants.XBOX_RUMBLE_SOFT)));

        BooleanSupplier readyToLaunch = ()-> mMechState == mReadyForLaunch;
        new Trigger(readyToLaunch).onTrue(new InstantCommand(()->Input.enableRumble(JoystickConstants.XBOX_RUMBLE_VIGEROUS)));

        BooleanSupplier readyToIntake = ()-> mMechState == mReadyForIntake;
        new Trigger(readyToIntake).onTrue(new InstantCommand(()->{
            Input.disableRumble();}));
    }

    public static MechState determineState() {
        if (mUseUltraInstinct) {
            return mUltraInstinct;
        }
        if (!getIntakeBeamBreak() && !getIndexerBeamBreak()) {
            return mReadyForIntake;
        }
        else if (getIntakeBeamBreak() && !getIndexerBeamBreak()) {
            if (AimState.HANDOFF.atState(mAimingSubsystem.getCurrentWristDegreees(),
                                         mAimingSubsystem.getCurrentElevatorDistance(),
                                         mLauncherSubsystem.getCurrentVelocity())) {
                return mReadyForHandoff;
            } else {
                return mIntaken;
            }
        }
        else if (getIndexerBeamBreak()) {
            if (isFlywheelAtSP() && isAimAtSP() && 
            !(Math.abs(mLauncherSubsystem.getCurrentVelocity()) < 500)) {
                return mReadyForLaunch;
            }
            else{
                return mReadyForAim;
            }
        }
        else{
            return mUltraInstinct;}
        
    }

    public void execute() {
        mMechState = determineState();

        runAction();

        if (Input.getX()) {
            mMechState.staticAutoAim().schedule();
            //This is for populating the event maps quickly
            // mMechState.setWristDeg(SmartDashboard.getNumber("Set Wrist Encoder Degrees", 0)).schedule();
        }
        else if (Input.getY()) {
            mMechState.speakerPosition().schedule();
        } 
        else if (Input.getA()) {
            mMechState.ampPosition().schedule();
        }
        else if (Input.getB()) {
            mMechState.handoffPosition().schedule();
            mMechState.brakeLauncher().schedule();            
        } else if(Input.getDPad() == Input.DPADDOWN){
            mMechState.staticAutoAim().schedule();
        }
        if (Input.getLeftBumper()) {
            mMechState.runIntakeMotors().schedule();
        } else if (Math.abs(Input.getRightTrigger()) > LauncherConstants.INDEX_TRIGGER_DEADZONE) {
            mMechState.overrideIntake((Input.getRightTrigger()-LauncherConstants.INDEX_TRIGGER_DEADZONE)
            *(Input.getReverseButton() ? -1 : 1)).schedule();
        } else{
            mMechState.brakeIntake().schedule();
        }

        if (Input.getRightBumper()) {
            mMechState.launch().schedule();

        } else if (Math.abs(Input.getLeftTrigger()) > LauncherConstants.INDEX_TRIGGER_DEADZONE) {
                mMechState.index(Math.pow((Input.getLeftTrigger()-LauncherConstants.INDEX_TRIGGER_DEADZONE)
                ,2)*(Input.getReverseButton() ? -1 : 1)).schedule();
        }

        if (Math.abs(Input.getLeftStickY()) > 0.1) {
            mMechState.controlWrist(Input.getLeftStickY()*AimingConstants.MAX_WRIST_TELEOP_INCREMENT*2).schedule();
        }
        if (Math.abs(Input.getRightStickY()) > 0.1) {
            mMechState.controlElevator(Input.getRightStickY()*AimingConstants.MAX_ELEVATOR_TELEOP_INCREMENT*1.5).schedule();
        }
        if (Input.getDPad() == Input.DPADUP) {
            mMechState.emergencyOuttake().schedule();
        }
        if (Input.getDPad() == Input.DPADRIGHT){
            mUseUltraInstinct = true;
        }
        if (Input.getDPad() == Input.DPADLEFT){
            mUseUltraInstinct = false;
        }



        SmartDashboard.putString("DesiredState", mDesiredState.name());
        SmartDashboard.putString("CurrentState", mMechState.getClass().getSimpleName());
        SmartDashboard.putBoolean("AutoAim Scheduled", mMechState.mStaticAutoAimCommand.isScheduled());
        SmartDashboard.putNumber("AutoAimPolynomial", AimingConstants.getPolynomialRegression());

    }

    //automatic actions
    public void runAction() {

        if (mMechState == mReadyForIntake) {
            Input.disableRumble();

            if (!mDesiredState.withinWristTolerance(mAimingSubsystem.getCurrentWristDegreees()) && mDesiredState != AimState.HANDOFF && mDesiredState != AimState.OUTTAKE){
                mMechState.index(0.3).schedule();
                return;
            }
             else{
                    mMechState.brakeLauncher().schedule();
                    mMechState.brakeIndexer().schedule();
                    if (!MechState.mHandoffPositionCommand.isScheduled()){
                        mMechState.handoffPosition().schedule();
                    }
            }
        }
        else if (mMechState == mIntaken) {
            mMechState.brakeIntake().schedule();
            mMechState.brakeIndexer().schedule();
            mMechState.brakeLauncher().schedule();
            if (!MechState.mHandoffPositionCommand.isScheduled()){
                mMechState.handoffPosition().schedule();
            }
        }
        else if (mMechState == mReadyForHandoff) {
            mMechState.brakeIntake().schedule();
            if (!MechState.mPreformHandoffCommand.isScheduled()){
                mMechState.preformHandoff().schedule();
            }
        }
        else if (mMechState == mReadyForAim) {
            
            mMechState.brakeIntake().schedule();
            // mMechState.brakeIndexer().schedule();
            
            //No Automation Yet
        }
        else if (mMechState == mReadyForLaunch) {
            mMechState.brakeIntake().schedule();
            // mMechState.brakeIndexer().schedule();
            //Need Operator Confirmation
            
        }
    }

    public static boolean getIntakeBeamBreak() {
        return IntakeSubsystem.pieceInIntake();
    }

    public static boolean getIndexerBeamBreak() {
        return mLauncherSubsystem.pieceInIndexer();
    }

    public static boolean isDriveBaseAligned() {
        if (mDesiredState == AimState.AUTO_AIM)
            return DefaultDriveCommand.getAlignedToSpeaker();
        return true; //V2: return LimelightOB.getNoteAlignmentCommand(swerve)
    }

    public static boolean isFlywheelAtSP() {
        return mLauncherSubsystem.isLauncherReady();
    }

    public static boolean isAimAtSP() {
        return mAimingSubsystem.atWristSetpoint() && mAimingSubsystem.atElevatorSetpoint();
    }

    public static MechState getMechState(){
        return mMechState;
    }

}
