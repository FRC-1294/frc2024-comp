package frc.robot.states.mech_states;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultMechCommand;
import frc.robot.constants.AimState;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.Autoaim;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForLaunch extends MechState {
    
    public ReadyForLaunch (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }
    
    @Override
    public Command launch() {
      //return mLauncherSubsystem.indexUntilNoteLaunchedCommand();
      if(DefaultMechCommand.mDesiredState == AimState.SUBWOOFER || DefaultMechCommand.mDesiredState == AimState.AMP){
        return MechState.mLaunchCommand;
      }
      return MechState.mStrangeIndexUntilLaunchCommand;
    }

    @Override
    public Command speakerPosition(){
      DefaultMechCommand.mDesiredState = AimState.SUBWOOFER;
      return MechState.mSpeakerPositionCommand;
    }

    @Override
    public Command ampPosition(){
      DefaultMechCommand.mDesiredState = AimState.AMP;
       return MechState.mAmpPositionCommand;
    }
    
    @Override
    public Command trapPosition(){
      DefaultMechCommand.mDesiredState = AimState.TRAP;
       return MechState.mTrapPositionCommand;
    }

    @Override
    public Command handoffPosition(){
      DefaultMechCommand.mDesiredState = AimState.HANDOFF;
       return MechState.mHandoffPositionCommand;
    }

    @Override
    public Command podiumPosition() {
      DefaultMechCommand.mDesiredState = AimState.PODIUM;
       return MechState.mPodiumPositionCommand;
    }
    @Override
    public Command index(double vel){
       return new InstantCommand(() -> mLauncherSubsystem.runIndexer(vel));
    }

    @Override
    public Command aimStatePosition(AimState aim){
        mAimStatePositionCommand = new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(aim),
        mLauncherSubsystem.waitUntilFlywheelSetpointCommand(aim));
        return mAimStatePositionCommand;
    }
    @Override
    public Command emergencyOuttake(){
        DefaultMechCommand.mDesiredState = AimState.OUTTAKE;
        mLaunchCommand.cancel();
       return new SequentialCommandGroup(new ParallelCommandGroup(mAimingSubsystem.waitUntilSetpoint(AimState.OUTTAKE),
            mLauncherSubsystem.waitUntilFlywheelSetpointCommand(AimState.OUTTAKE)),
            new InstantCommand(()->mLauncherSubsystem.runIndexer(-0.6))); 
    }

    @Override
    public Command staticAutoAim(){
      DefaultMechCommand.mDesiredState = AimState.AUTO_AIM;
        return MechState.mStaticAutoAimCommand;
    }
}

