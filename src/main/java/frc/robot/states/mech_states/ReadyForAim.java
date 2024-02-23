package frc.robot.states.mech_states;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForAim extends MechState {

    public ReadyForAim (LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

    @Override
    public void speakerPosition(){
        new InstantCommand(()->{
            mAimingSubsystem.setDesiredSetpoint(AimState.SPEAKER);
            mLauncherSubsystem.setLauncherMode(LauncherMode.SPEAKER);}
        ).schedule();
    }

    @Override
    public void ampPosition(){
        new InstantCommand(()->{
            mAimingSubsystem.setDesiredSetpoint(AimState.AMP);
            mLauncherSubsystem.setLauncherMode(LauncherMode.AMP);}
        ).schedule();
    }
    
    @Override
    public void trapPosition(){
        new InstantCommand(()->{
            mAimingSubsystem.setDesiredSetpoint(AimState.TRAP);
            mLauncherSubsystem.setLauncherMode(LauncherMode.TRAP);}
        ).schedule();
    }
}
