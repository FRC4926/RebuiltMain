package frc.robot.constants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class VisionConstants {
    public record CameraWrapperConstants(String name, Transform3d robotToCamera, double trustFactor) {};
    public static final CameraWrapperConstants[] camConstants = {
        new CameraWrapperConstants("ArducamTop",
            new Transform3d(new Translation3d(4.828*0.0254, -11.804*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,Math.PI/3).unaryMinus()), .1),
        new CameraWrapperConstants("Right",
            new Transform3d(new Translation3d(2.184*0.0254, -11.804*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,2*Math.PI/3).unaryMinus()), 1),
        new CameraWrapperConstants("ArducamBack",
            new Transform3d(new Translation3d(2.184*0.0254, 11.804*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,-2*Math.PI/3).unaryMinus()), 1),// 1.25
        // new CameraWrapperConstants("ArducamRight",
        //     new Transform3d(new Translation3d(-11.804*0.0254, -4.828*0.0254, 16*0.0254), new Rotation3d(0,-15*Math.PI/180,-Math.PI/3)), 1),
        //  new CameraWrapperConstants("Limelight",
        //          new Transform3d(new Translation3d(7*0.0254,0 , 10.5*0.0254), new Rotation3d(0,-15*Math.PI/18,Math.PI)), 2) //1.25
    };

    public static final double kalmanPositionStdDevCoeefficient = 0.15; //0.6
    public static final double kalmanRotationStdDev = 0.99;

    public static final double maximumAmbiguity = 0.25; //0.25

    
    public static final PIDController objectDetectionDrivePIDController = new PIDController(0.25, 0, 0); //0.25

    public static final PIDController objectDetectionRotationPIDController = new PIDController(0.2067, 0, 0);
    public static final double objectDetectionRotationTolerance = 0.7;
    public static final double objectDetectHorizonPitch = 20;

    public static final double timeThres = 1;

    public static final double yawWeight = 0.03;
    public static final double areaWeight = 10;
}