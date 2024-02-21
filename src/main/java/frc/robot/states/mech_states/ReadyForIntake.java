package frc.robot.states.mech_states;

import frc.robot.constants.AimingConstants.AimState;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.LauncherConstants.LauncherMode;
import frc.robot.states.MechState;
import frc.robot.subsystems.AimingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LauncherSubsystem;

public class ReadyForIntake extends MechState {

    public ReadyForIntake(LauncherSubsystem launcherSubsystem,AimingSubsystem aimingSubsystem,IntakeSubsystem intakeSubsystem) {
        super(launcherSubsystem,aimingSubsystem,intakeSubsystem);
    }

     @Override
    public void intake() {
        mIntakeSubsystem.intakeAtSpeed(IntakeConstants.ACTIVE_INTAKE_SPEED);
    }

    @Override
    public void handoffPosition(){
        mAimingSubsystem.setDesiredSetpoint(AimState.HANDOFF);
        mLauncherSubsystem.setLauncherMode(LauncherMode.OFF);
    }
}
