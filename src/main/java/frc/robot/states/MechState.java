package frc.robot.states;

import javax.swing.GroupLayout.SequentialGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutonomousCommands.Handoff;
import frc.robot.commands.AutonomousCommands.StrangeIndexUntilLaunchCommand;
import frc.robot.constants.AimState;
import frc.robot.constants.AimingConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public abstract class MechState {
    protected final LauncherSubsystem mLauncherSubsystem;
    protected final IntakeSubsystem mIntakeSubsystem;
    protected final AimingSubsystem mAimingSubsystem;
    public static Command mHandoffPositionCommand;
    public static Command mSpeakerPositionCommand;
    public static Command mAmpPositionCommand;
    public static Command mTrapPositionCommand;
    public static Command mPreformHandoffCommand;
    public static Command mLaunchCommand;
    public static Command mPodiumPositionCommand;
    public static Command mBrakeIndexerCommand;
    public static Command mStaticAutoAimCommand;
    public static Command mStrangeIndexUntilLaunchCommand;

    public Command mAimStatePositionCommand;
    public static Command mCalculationAutoAimCommand;
    

    protected MechState (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        mLauncherSubsystem = launcherSubsystem;
        mIntakeSubsystem = intakeSubsystem;
        mAimingSubsystem = aimingSubsystem;

        
        mHandoffPositionCommand = mAimingSubsystem.waitUntilSetpoint(AimState.HANDOFF);
        mSpeakerPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.SUBWOOFER), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.SUBWOOFER));
        mAmpPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.AMP),mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.AMP));
        mTrapPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.TRAP), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.TRAP));
        mPreformHandoffCommand = new Handoff(mIntakeSubsystem, mLauncherSubsystem);
        mPodiumPositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.PODIUM), mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.PODIUM));
        mLaunchCommand = new SequentialCommandGroup(mLauncherSubsystem.indexUntilNoteLaunchedCommand());
        mBrakeIndexerCommand = new InstantCommand(()->mLauncherSubsystem.stopIndexer(),mLauncherSubsystem);
        mStaticAutoAimCommand = new ParallelCommandGroup(
            mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.PODIUM),
            mAimingSubsystem.waitUntilAutoAimSetpointTracked()
        );
        mStrangeIndexUntilLaunchCommand = new SequentialGroup(StrangeIndexUntilLaunchCommand(launcherSubsystem),mLaunchCommand);
        // new ParallelCommandGroup(mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.PODIUM), 
        // mAimingSubsystem.waitUntilWristSetpoint(() -> AimingConstants.getPolynomialRegression(FieldConstants.getSpeakerDistance())));
    }

    public Command setLauncherSpeed(AimState state) {
        return new PrintCommand("Can't set launcher speed now");
    }

    public Command brakeLauncher() {
        return new InstantCommand(()->mLauncherSubsystem.stopLauncher(),mLauncherSubsystem);
    }
    public Command brakeIndexer() {
       return mBrakeIndexerCommand;
    }
    public Command brakeIntake() {
       return new InstantCommand(()->mIntakeSubsystem.stopMotors(),mIntakeSubsystem);
    }
    public Command emergencyOuttake(){
        return new InstantCommand();
    }

    public Command runIntakeMotors(){
        return new InstantCommand();
    }

    public Command preformHandoff(){
        return new InstantCommand();
    }

    public Command index(double vel){
        return new InstantCommand();
    }

    public Command overrideIntake(double vel){
        return new InstantCommand();
    }

    public Command launch() {
        return new InstantCommand();
    }

    public Command controlWrist(double increment) {
        return new InstantCommand();
    }

    public Command controlElevator(double increment) {
        return new InstantCommand();
    }

    public Command ClimbExtendedState() {
        return new InstantCommand();
    }
    
    public Command ClimbRetractedState() {
        return new InstantCommand();
    }

    public Command setWristSP(AimState state) {
        return new InstantCommand();
    }
    
    public Command handoffPosition(){
        return new InstantCommand();
    }

    public Command speakerPosition(){
        return new InstantCommand();
    }

    public Command ampPosition(){
        return new InstantCommand();
    }
    
    public Command trapPosition(){
        return new InstantCommand();
    }

    public Command aimStatePosition(AimState state){
        return new InstantCommand();
    }

    public Command resetEncoders() {
        return new InstantCommand();
    }

    public Command podiumPosition(){
        return new InstantCommand();
    }

    public Command staticAutoAim(){
        return new InstantCommand();
    }

    public Command calculationBasedAutoaim() {
        return new InstantCommand();
    }
    public Command setWristDeg(double deg){
        return new InstantCommand();
    }

}
