package frc.robot.states.mech_states;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class UltraInstinct extends MechState {
    public UltraInstinct (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem){ 
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
        
    }

    @Override
    public Command setLauncherSpeed(AimState state) {
       return new InstantCommand(()->mLauncherSubsystem.setLauncherState(state));
    }
    @Override
    public Command brakeLauncher() {
       return new InstantCommand(()->mLauncherSubsystem.stopLauncher());
    }

    @Override
    public Command runIntakeMotors() {
       return new InstantCommand(() -> {
        mIntakeSubsystem.innerMotorAtSpeed(IntakeConstants.INNER_INTAKE_SPEED_AQUIRE);
        mIntakeSubsystem.outerMotorAtSpeed(IntakeConstants.OUTER_INTAKE_SPEED_ACTIVE);},mIntakeSubsystem);
    }

    @Override
    public Command index(double vel){
      return  new InstantCommand(() -> mLauncherSubsystem.runIndexer(vel));
    }
    
    @Override
    public Command overrideIntake(double vel){
       return new InstantCommand(() -> mIntakeSubsystem.intakeMotorsAtSpeed(vel));
    }

    @Override
    public Command launch() {
       return mStrangeIndexUntilLaunchCommand;
    }

    @Override
    public Command controlWrist(double increment) {
       return new InstantCommand(()->mAimingSubsystem.changeDesiredWristRotation(increment),mAimingSubsystem);
    }

    @Override
    public Command controlElevator(double increment) {
       return new InstantCommand(()->mAimingSubsystem.changeDesiredElevatorPosition(increment),mAimingSubsystem);
    }

    @Override
    public Command ClimbExtendedState() {
       return new InstantCommand(()->mAimingSubsystem.setDesiredSetpoint(AimState.CLIMB_UP));
    }

    @Override
    public Command ClimbRetractedState() {
       return new InstantCommand(()->mAimingSubsystem.setDesiredSetpoint(AimState.CLIMB_DOWN));
    }

    @Override
    public Command setWristSP(AimState state) {
       return new InstantCommand(()->mAimingSubsystem.setDesiredWristRotation(state.mWristAngleDegrees, state.mWristToleranceDegrees),mAimingSubsystem);
    }

    @Override
    public Command preformHandoff(){
       return MechState.mHandoffPositionCommand;
    }

    @Override
    public Command speakerPosition(){
       return mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.PODIUM);
    }

    @Override
    public Command ampPosition(){
       return MechState.mAmpPositionCommand;
    }
    
    @Override
    public Command trapPosition(){
       return MechState.mTrapPositionCommand;
    }

    @Override
    public Command handoffPosition(){
       return MechState.mHandoffPositionCommand;
    }

    @Override
    public Command setWristDeg(double deg){
      return new InstantCommand(()->mAimingSubsystem.setDesiredWristRotation(deg));
   }
}
