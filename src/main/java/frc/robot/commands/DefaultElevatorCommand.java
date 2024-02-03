// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JoystickConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Input;

public class DefaultElevatorCommand extends Command {
  /** Creates a new DefaultElevatorCommand. */
  private Elevator mElevator;
  public DefaultElevatorCommand(Elevator elevator) {
    mElevator = elevator;
    addRequirements(mElevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (Math.abs(Input.mXBox.getLeftY())>JoystickConstants.XBOX_Y_DEADZONE){
      mElevator.setVelocity(Input.mXBox.getLeftY()*0.1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
