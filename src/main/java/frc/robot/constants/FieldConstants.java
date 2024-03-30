package frc.robot.constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveSubsystem;

public class FieldConstants {
    public static enum Blue {
        SPEAKER(new Pose3d(new Translation3d(0, 5.6, 0.0), new Rotation3d(0.0, 0.0, 0.0)));

        private final Pose3d pose;

        Blue(Pose3d pos) {
            pose = pos;
        }
        /*
         * Return the BACK CENTER location of the object
         */
        public Pose3d getPose() {
            return pose;
        }
    }

    public static enum Red {
        SPEAKER(new Pose3d(new Translation3d(16.5, 5.6, 0.0), new Rotation3d(0.0, 0.0, 0.0)));

        private final Pose3d pose;

        Red(Pose3d pos) {
            pose = pos;
        }
        /*
         * Return the BACK CENTER location of the object
         */
        public Pose3d getPose() {
            return pose;
        }
    }

    public static final double SPEAKER_LENGTH_METERS = 0.36;
    public static final double SPEAKER_WIDTH_METERS = 1.05;

    public static double getSpeakerDistance(Pose2d robotPose) {
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return Red.SPEAKER.getPose().toPose2d().minus(robotPose).getTranslation().getNorm();            
        }
        return Blue.SPEAKER.getPose().toPose2d().minus(robotPose).getTranslation().getNorm();

    }

    public static double getSpeakerDistance() {
        Pose2d robotPose = SwerveSubsystem.getRobotPose();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            return Red.SPEAKER.getPose().toPose2d().minus(robotPose).getTranslation().getNorm();            
        }
        return Blue.SPEAKER.getPose().toPose2d().minus(robotPose).getTranslation().getNorm();

    }
}
