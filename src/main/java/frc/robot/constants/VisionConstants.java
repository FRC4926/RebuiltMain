package frc.robot.constants;

import java.util.ArrayList;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public record CameraWrapperConstants(String name, Transform3d robotToCamera, double trustFactor) {};
    public static final CameraWrapperConstants[] camConstants = {
        new CameraWrapperConstants("BackLeft",
            new Transform3d(new Translation3d(-0.32964297, 0.280335875, 0.532235), new Rotation3d(0,0.0, 165*Math.PI/180)), 1),
        new CameraWrapperConstants("BackRight",
            new Transform3d(new Translation3d(-0.32964297, -0.280335875, 0.532235), new Rotation3d(0,0.0, 195*Math.PI/180)), 1),
        new CameraWrapperConstants("Arducam_OV9281_USB_Camera",
            new Transform3d(new Translation3d(-0.05190144, -0.17368266, 0.494284), new Rotation3d(0, 30*Math.PI/180, -15*Math.PI/180)), 1),
         new CameraWrapperConstants("FrontLeft",
            new Transform3d(new Translation3d(-0.05190144, 0.17368266, 0.494284), new Rotation3d(0, 30*Math.PI/180, 15*Math.PI/180)), 1)
    };

    public static final double kalmanPositionStdDevCoeefficient = 0.33; //0.3
    public static final double kalmanRotStdDevCoeefficient = Double.POSITIVE_INFINITY; //0.6

    // public static final double kalmanRotationStdDev = 0.99;

    public static final double maximumAmbiguity = 0.25; //0.25

    
    public static final PIDController objectDetectionDrivePIDController = new PIDController(0.25, 0, 0); //0.25

    public static final PIDController objectDetectionRotationPIDController = new PIDController(0.2067, 0, 0);
    public static final double objectDetectionRotationTolerance = 0.7;
    public static final double objectDetectHorizonPitch = 20;

    public static final double objectDetectMinBias = 0.1;
    public static final double objectDetectAreaThreshold = 1.2;

    public static final double timeThres = 1;

    public static final double yawWeight = 0.03;
    public static final double areaWeight = 10;

    public static final PathConstraints pathOnTheFlyConstraints = new PathConstraints(
            4,5,
            Units.degreesToRadians(540), Units.degreesToRadians(720));
}