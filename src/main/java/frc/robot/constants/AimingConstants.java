// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import java.lang.reflect.Field;
import java.util.Optional;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Util.PIDParameters;
import frc.robot.subsystems.AimingSubsystem.AimingMotorMode;

// AimingConstants class is used to store all IDs & constants of the Elevator & Wrist components
public class AimingConstants {
    
    // Teleop Constant
    public static final double MAX_ELEVATOR_TELEOP_INCREMENT = 0.003;
    public static final double MAX_WRIST_TELEOP_INCREMENT = 0.2;
    
    
    // PID Constants
    public static final PIDParameters mElevatorPIDConstants = new PIDParameters(13, 0, 0);
    public static final PIDParameters mWristPIDConstants = new PIDParameters(0.015, 0.001, 0.0, 0, 0);
    public static final double WRIST_KG = 0.03;
    public static final double ELEVATOR_FEEDFORWARD_CONSTANT = 0.05;
    public static final double SPARK_THROUGHBORE_GEAR_RATIO = 1;


    public static final AimingMotorMode INITIAL_MOTOR_MODE = AimingMotorMode.BRAKE;

    // TODO: Calculate the ratio for the competition robot
    // Issue #281 has been created for the same. Link to the issue: https://github.com/FRC-1294/frc2024/issues/281
    public static final double ELEVATOR_ROTATIONS_TO_METERS = 0.45085 / 367.8760986328125;
	public static final double ELEVATOR_TOLERANCE_IN = 0.00;
    public static final double WRIST_TOLERANCE_DEG = 2;

    // Soft Limits
    public static final double MAX_ELEVATOR_DIST_METERS = 0.45085;
    public static final double MIN_ELEVATOR_DIST_METERS = 0.01; 
    public static final double MAX_ELEVATOR_EXTENSION_VELOCITY = 0.1; //TBD

    public static final double MAX_WRIST_ROTATION = 110;
    public static final double MIN_WRIST_ROTATION_DEG = 0;
    public static final double MAX_WRIST_ROTATION_VELOCITY = 0;

    public static final double MAX_WRIST_PID_CONTRIBUTION = 0.6;
    public static final double MAX_ELEVATOR_PID_CONTRIBUTION = 1;


    // inaccurate values
    public static final double REST_LAUNCH_ANGLE = 50.429; // rest is when wrist rotation is zero degrees
    public static final double NOTE_EXIT_SPEED = 26.00;
    public static final double AREA1 = (12.0/39.37) * (2.0/39.37) + (Math.PI * Math.pow(1/39.37, 2));
    public static final double AREA2 = Math.PI * (.3556*.3556) - Math.PI * (.254*.254);
    public static final double FLUID_DENSITY = 1.293;
    public static final double MASS = 0.2353010419;

    public static final double DRAG_COEFFICIENT = 0.0;
    public static final double WRIST_D1 = Units.inchesToMeters(8.3);
    public static final double WRIST_D2 = .00;
    public static final double WRIST_BEND_ANGLE = Units.degreesToRadians(REST_LAUNCH_ANGLE); //Units.degreesToRadians(50.429);
    public static final Pose3d BLUE_SPEAKER_POS = new Pose3d(.3, 5.5, 2.1, new Rotation3d());
    public static final Pose3d RED_SPEAKER_POS = new Pose3d(16.2, 5.5, 2.1, new Rotation3d());
    public static final double AUTOAIM_TIME_LOOKAHEAD = 0.0;

    // ID'
    public static final int LEFT_ELEVATOR_SPARK_ID = 33; //Done
    public static final int RIGHT_ELEVATOR_SPARK_ID = 32; //Done

    public static final int LEFT_WRIST_SPARK_ID = 34; //Done
    public static final int RIGHT_WRIST_SPARK_ID = 35; //Done

    public static final int WRIST_THROUGHBORE_ENCODER_ID = 0;
    public static final double WRIST_THROUGHBORE_GEAR_RATIO = 1;
    public static final double WRIST_THROUGHBORE_ENCODER_OFFSET = 271+1+62.2+1-24.2-54.1-89.2-4.1+0.3-0.15-.7-0.48+298.2-177.44-0.6+0.65+9.2-94.2-0.62+255.0-95.0;
    public static final double COG_OFFSET = 22;
    public static final double WRIST_KS = 0.005;


    // If false, then motors are physically inverted
    public static final boolean ELEVATOR_LEFT_IS_NORMAL = true;
    public static final boolean WRIST_LEFT_IS_NORMAL = false;

    public static final int CONNECTION_THRESH_HZ = 945;

    public static final InterpolatingDoubleTreeMap AIM_MAP = new InterpolatingDoubleTreeMap();
    public static final double MAX_SHOT_DIST_METERS = 4.5;
    public static final double MAX_WRIST_ACCURACY_DEG = 0.5;
    private static final double TOLERANCE_DAMPENING_CONSTANT = 0.15;
    private static final double ROTATION_PADDING_METERS = 0.15;

    public static void populate_aim_map(){
        AimingConstants.AIM_MAP.put(AimState.MIDNOTE.mWristAngleDegrees, AimState.MIDNOTE.mRadialDistanceMeters);
        AimingConstants.AIM_MAP.put(AimState.SUBWOOFER.mWristAngleDegrees, AimState.SUBWOOFER.mRadialDistanceMeters);
        AimingConstants.AIM_MAP.put(AimState.LINE.mWristAngleDegrees, AimState.LINE.mRadialDistanceMeters);
        AimingConstants.AIM_MAP.put(AimState.WING.mWristAngleDegrees, AimState.WING.mRadialDistanceMeters);
    }

    public static double getPolynomialRegression(){
        if (FieldConstants.getSpeakerDistance()>4.8){
            return -15 + 16.1*FieldConstants.getSpeakerDistance() - 1.65*Math.pow(FieldConstants.getSpeakerDistance(), 2) + 0.0683*Math.pow(FieldConstants.getSpeakerDistance(), 3)
            + (16.1 - 3.3*FieldConstants.getSpeakerDistance() + 0.2049*Math.pow(FieldConstants.getSpeakerDistance(),2))*0.0;
        }else{
            return -7.26 + 7.65*FieldConstants.getSpeakerDistance() + 1.27*Math.pow(FieldConstants.getSpeakerDistance(), 2) - 0.25*Math.pow(FieldConstants.getSpeakerDistance(), 3)
            + (7.625 + 2.54*FieldConstants.getSpeakerDistance() - 0.75*Math.pow(FieldConstants.getSpeakerDistance(),2))*0.43;
        }
        
        //return -25+16.3*dist+0.757*Math.pow(dist, 2)-0.349*Math.pow(dist, 3);
    }

    public static double getAutoAimWristToleranceDegrees(){
        // if (FieldConstants.getSpeakerDistance()>4.6){
        //     return 1;
        //     // return Math.max((16.1 - 3.3*FieldConstants.getSpeakerDistance() + 0.2049*Math.pow(FieldConstants.getSpeakerDistance(),2))
        //     // * TOLERANCE_DAMPENING_CONSTANT,MAX_WRIST_ACCURACY_DEG);
        // }else{
        //     return Math.max((7.625 + 2.54*FieldConstants.getSpeakerDistance() - 0.75*Math.pow(FieldConstants.getSpeakerDistance(), 2))
        //     * TOLERANCE_DAMPENING_CONSTANT,MAX_WRIST_ACCURACY_DEG);
        // }
        return 1;
    }

    public static double getSwerveAlignmentToleranceDeg(){
        return 2.0;
        //return Math.toDegrees(Math.atan2((FieldConstants.SPEAKER_WIDTH_METERS/2)-AimingConstants.ROTATION_PADDING_METERS,FieldConstants.getSpeakerDistance()));
    }
} 
