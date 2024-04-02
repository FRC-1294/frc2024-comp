// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.SwerveSubsystem;

/** Add your docs here. */
public enum AimState {
    AUTOAIM(-1, -1, 12000, 500),
    
    SUBWOOFER(0,3, 7000, 2000), //Tolerance TBD
    AMP(100,5, 3000, 400), //Everything TBD
    TRAP(110,-1,-1,0,0,0), //Everything TBD
    HANDOFF(0,2,-1,-1), //Tolerance TBD
    OUTTAKE(30,3,-1000,100),
    PODIUM(32, 2, 12000, 1000),

    LINE(0,0,0,0), //Everything TBD
    MIDNOTE(32, 2, 10000, 1000), //Everything TBD
    WING(0,0,0,0), //Everything TBD
    CLIMB_UP(-1,-1,-1,-1,AimingConstants.MAX_ELEVATOR_DIST_METERS-0.2,0.1,-1,-1),
    CLIMB_DOWN(-1,-1,-1,-1,AimingConstants.MIN_ELEVATOR_DIST_METERS,0.1,-1,-1),
    AUTO_AIM(-1,-1,-1,-1);

    public final double mWristAngleDegrees;
    public final double mRadialDistanceMeters;
    public final double mElevatorHeightMeters;
    public final double mPositionToleranceMeters;
    public final double mWristToleranceDegrees;
    public final double mElevatorToleranceMeters;
    public final double mLauncherSetpointRPM;
    public final double mLauncherToleranceRPM;

    AimState(double wristAngleDeg, double radialDistanceMeters, double shotToleranceMeters,
     double wristToleranceDegrees, double elevatorHeight, double elevatorTolerance, double launcherSetpoint, double launcherTolerance) {
        mWristAngleDegrees = wristAngleDeg;
        mRadialDistanceMeters = radialDistanceMeters;
        mPositionToleranceMeters = shotToleranceMeters;
        mWristToleranceDegrees = wristToleranceDegrees;
        mElevatorHeightMeters = elevatorHeight;
        mElevatorToleranceMeters = elevatorTolerance;
        mLauncherSetpointRPM = launcherSetpoint;
        mLauncherToleranceRPM = launcherTolerance;

    }

    AimState(double wristAngleDeg, double radialDistanceMeters, double shotToleranceMeters,
     double wristToleranceDegrees, double launcherSetpoint, double launcherTolerance) {
        mWristAngleDegrees = wristAngleDeg;
        mRadialDistanceMeters = radialDistanceMeters;
        mPositionToleranceMeters = shotToleranceMeters;
        mWristToleranceDegrees = wristToleranceDegrees;
        mElevatorHeightMeters = -1;
        mElevatorToleranceMeters = -1;        
        mLauncherSetpointRPM = launcherSetpoint;
        mLauncherToleranceRPM = launcherTolerance;
    }

    AimState(double wristAngleDeg,double wristToleranceDegrees, double launcherSetpoint, double launcherTolerance) {
        mWristAngleDegrees = wristAngleDeg;
        mWristToleranceDegrees = wristToleranceDegrees;
        mLauncherSetpointRPM = launcherSetpoint;
        mLauncherToleranceRPM = launcherTolerance;

        mRadialDistanceMeters = -1;
        mElevatorHeightMeters = -1;
        mElevatorToleranceMeters = -1;
        mPositionToleranceMeters = -1;

    }

    public boolean atState(double curWristAngle, double curElevatorHeight, double curLauncherSpeed, double otherSped){
        //return true if the current state is within the tolerance of the desired state ignoring the parameters that are -1
        return withinWristTolerance(curWristAngle) && withinLauncherTolerance(curLauncherSpeed, otherSped);
    }

    // private double[] getPolarCoordsFromXY(Pose2d curSwervePose){
    //     double [] coords = new double[2];
    //     //return the polar coordinates in the form of (r,theta) from the 
    //     return coords;
    // } //Not used

    public boolean withinWristTolerance(double curWristAngle){
        if (this == AUTO_AIM){
            return Math.abs(curWristAngle-AimingConstants.getPolynomialRegression())<=AimingConstants.getAutoAimWristToleranceDegrees();
        }
        return Math.abs(curWristAngle-mWristAngleDegrees)<=mWristToleranceDegrees || mWristAngleDegrees == -1;
    }
        
    public boolean withinElevatorTolerance(double curElevatorHeight){
        return (Math.abs(curElevatorHeight-mElevatorHeightMeters)<=mElevatorToleranceMeters || mElevatorHeightMeters == -1);
    }

    public boolean withinLauncherTolerance(double curLauncherSpeed, double curOtherSped){
        return (Math.abs(curLauncherSpeed-mLauncherSetpointRPM)<=mLauncherToleranceRPM && Math.abs(curOtherSped-mLauncherSetpointRPM)<=mLauncherToleranceRPM) || mLauncherSetpointRPM == -1;
    }

    public boolean withinSwerveTolerance(Pose2d curServePose){
        return true;
    }
    public boolean withinSwerveYawTolerance(){
        return true;
    }

}
