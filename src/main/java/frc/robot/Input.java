// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.constants.JoystickConstants;


// Input Class For Joystick/Controller Input Functions
public class Input {
  private Input() {
    throw new IllegalStateException("Input Class");
  }

  private static final Joystick mRotJoystick = new Joystick(JoystickConstants.ROT_JOYSTICK_PORT);
  private static final Joystick mTransJoystick = new Joystick(JoystickConstants.TRANS_JOY_PORT);
  private static final XboxController mXboxController = new XboxController(JoystickConstants.XBOX_PORT);

  public static final int DPADUP = 0;
  public static final int DPADRIGHT = 90;
  public static final int DPADDOWN = 180;
  public static final int DPADLEFT = 270;


  public static boolean resetGyro() {
    return mRotJoystick.getRawButton(3);
  }

  public static boolean alignAmp(){
    return mRotJoystick.getRawButton(2);
  }

  public static boolean atanAlignSpeaker() {
    return mRotJoystick.getRawButton(5);//TBD
  }
  public static boolean resetOdo() {
    // return mTransJoystick.getRawButton(3);
    return false;
  }

  public static void enableRumble(double intensity){
    mXboxController.setRumble(RumbleType.kRightRumble, intensity);
  }

  public static void disableRumble(){
    mXboxController.setRumble(RumbleType.kRightRumble, 0);
  }

  public static void enableLeftRumble(double intensity){
    mXboxController.setRumble(RumbleType.kLeftRumble, intensity);
  }

  public static void disableLeftRumble(){
    mXboxController.setRumble(RumbleType.kLeftRumble, 0);
  }

  public static double getJoystickX() {
    return mTransJoystick.getX();
  }

  public static double getJoystickY() {
    return mTransJoystick.getY();
  }

  public static boolean getNoteLock(){
    return mRotJoystick.getRawButton(4);
  }

  public static boolean getLeftBumper() {
    return mXboxController.getLeftBumper();
  }

  public static boolean getRightBumper() {
    return mXboxController.getRightBumperPressed();
  }

  public static double getLeftTrigger() {
    return mXboxController.getLeftTriggerAxis();
  }

  public static double getRightTrigger() {
    return mXboxController.getRightTriggerAxis();
  }

  public static double getRot() {
    return mRotJoystick.getX();
  }

  public static boolean getA() {
    return mXboxController.getAButtonPressed();
  }

  public static boolean getB() {
    return mXboxController.getBButtonPressed();
  }

  public static boolean getX() {
    return mXboxController.getXButtonPressed();
  }

  public static boolean getY() {
    return mXboxController.getYButtonPressed();
  }

  public static double getDPad() {
    return mXboxController.getPOV();
  }
  
  public static double getLeftStickY() {
    return -mXboxController.getLeftY();
  }

  public static double getRightStickY() {
    return -mXboxController.getRightY();
  }

  public static boolean getResetGyro() {
    return mRotJoystick.getRawButton(3);
  }

  public static boolean getPrecisionToggle() {
    return mTransJoystick.getTriggerPressed();
  }

  public static boolean getVelLock() {
    return mRotJoystick.getTrigger();
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

  public static boolean overrideIntakeBeamBreak() {
    return mXboxController.getRightStickButton();
  }

  public static boolean getReverseButton(){
    return mXboxController.getRawButton(7);
  }
}

