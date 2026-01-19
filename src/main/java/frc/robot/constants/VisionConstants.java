package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public record CameraWrapperConstants(String name, Transform3d robotToCamera, double trustFactor) {};
    public static final CameraWrapperConstants[] camConstants = {
        new CameraWrapperConstants("ArducamFront",
            new Transform3d(new Translation3d(11.804*0.0254, 4.828*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,Math.PI/3)), 1),
        new CameraWrapperConstants("ArducamBack",
            new Transform3d(new Translation3d(11.804*0.0254, 2.184*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,2*Math.PI/3)), 1),
        new CameraWrapperConstants("ArducamLeft",
            new Transform3d(new Translation3d(-11.804*0.0254, -2.184*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,-2*Math.PI/3)), 1.25),
        new CameraWrapperConstants("ArducamRight",
            new Transform3d(new Translation3d(-11.804*0.0254, -4.828*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,-Math.PI/3)), 1),
        new CameraWrapperConstants("limelight",
                new Transform3d(new Translation3d(0, 7*0.0254, 10.5*0.0254), new Rotation3d(0,-15*Math.PI/18,Math.PI)), 1.25)
    };
    public static final double kalmanPositionStdDevCoeefficient = 0.15;
    public static final double kalmanRotationStdDev = 0.99;

    public static final double maximumAmbiguity = 0.25;
}
