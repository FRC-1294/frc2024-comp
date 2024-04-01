// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
  private VisionConstants() {
    throw new IllegalStateException("Constants Class");
  }

  public static final double ROT_SPEED = 0.25;
  public static final String LIMELIGHT_NAME = "limelight";

  // These are the names as they appear in photonvision of the cameras. These can be changed in the
  // photonvision dashboard
  public static final String CAMERA_NAME_FRONT = "AgniVision2";
  public static final String CAMERA_NAME_BACK = "Arducam_OV9281_USB_Camera";

  // These are tunable constants for the reliability of odometry and vision measurements in the form
  // of a vector of (x, y, theta), in meters, meters, and radians respectively
  public static final Matrix<N3, N1> STATE_STD_DEVS =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(2));
  public static final Matrix<N3, N1> SINGLE_TAG_VISION_MEASUREMENTS_STD_DEVS =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(20));
  public static final Matrix<N3, N1> MULTI_TAG_VISION_MEASUREMENTS_STD_DEVS =
      VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(20));


  public static final Transform3d ROBOT_TO_CAM_VEC_FRONT = new Transform3d(
      new Translation3d(-0.2337199-0.0061722,
                        -0.22485,
                        0.3361519+0.010668
                        ), 
      new Rotation3d()
        .rotateBy(new Rotation3d(0, Math.toRadians(-30), 0))
        .rotateBy(new Rotation3d(0, 0, Math.toRadians(180)))
        
    );
  public static final Transform3d ROBOT_TO_CAM_VEC_BACK =
      new Transform3d(new Translation3d(-0.1287653508, -0.205200-0.0123, Units.inchesToMeters(15)),
        new Rotation3d(0, Math.toRadians(-1.8), Math.toRadians(-90)));

  public static final double SINGLE_TAG_AMBIGUITY_THRESH = 0.15;
}
