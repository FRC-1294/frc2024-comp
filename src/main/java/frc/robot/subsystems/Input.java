// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.XboxController;
>>>>>>> 074282cdf93a863b1c1f198143d038490f368d0d
import frc.robot.constants.JoystickConstants;

// Input Class For Joystick/Controller Input Functions
public class Input {
  private Input() {
    throw new IllegalStateException("Input Class");
  }

  private static final Joystick mRotJoystick = new Joystick(JoystickConstants.ROT_JOYSTICK_PORT);
  private static final Joystick mTransJoystick = new Joystick(JoystickConstants.TRANS_JOY_PORT);
<<<<<<< HEAD
=======
  private static final XboxController mXboxController = new XboxController(JoystickConstants.XBOX_CONTROLLER_PORT);
>>>>>>> 074282cdf93a863b1c1f198143d038490f368d0d


  public static final int DPADUP = 0;
  public static final int DPADRIGHT = 90;
  public static final int DPADDOWN = 180;
  public static final int DPADLEFT = 270;


  public static boolean resetGyro() {
    return mRotJoystick.getRawButton(3);
  }

  public static boolean resetOdo() {
    return mTransJoystick.getRawButton(3);
  }

  public static double getJoystickX() {
    return mTransJoystick.getX();
  }

  public static double getJoystickY() {
    return mTransJoystick.getY();
  }

  public static double getRot() {
    return mRotJoystick.getX();
  }

  public static boolean getResetGyro() {
    return mRotJoystick.getRawButton(3);
  }

  public static boolean getPrecisionToggle() {
    return mTransJoystick.getTriggerPressed();
  }

  public static boolean getIncPID() {
    return mRotJoystick.getRawButton(5);
  }

  public static boolean getDecPID() {
    return mRotJoystick.getRawButton(4);
  }

  public static boolean togglePIDTuning() {
    return mRotJoystick.getTriggerReleased();
  }
<<<<<<< HEAD
=======

  public static boolean getIntake() {
    return mXboxController.getLeftBumper();
  }

  public static boolean getOnePiece() {
    return mXboxController.getRightStickButton();
  }
>>>>>>> 074282cdf93a863b1c1f198143d038490f368d0d
}

