package frc.robot.subsystems;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.CameraWrapper;

public class ObjectDetectionSubsytem extends SubsystemBase {

    private final PIDController driveController =
            VisionConstants.objectDetectionDrivePIDController;
    private final PIDController rotationController =
            VisionConstants.objectDetectionRotationPIDController;

    private CameraWrapper camera;
    private double bias = 0.0;
    private double softBias = 0.0;
    private boolean state = false;
    private double avgArea = 0;
    public ObjectDetectionSubsytem() {
        //I know its weird but surely its fine
        camera = new CameraWrapper("ArducamColor", new Transform3d(new Translation3d(11.804*0.0254, 0, 20*0.0254), new Rotation3d(0,-15*Math.PI/180,Math.PI/3)), FieldConstants.tagLayout, false, 0, RobotContainer.drivetrain::addVisionMeasurement);
        driveController.setTolerance(0);
        rotationController.setTolerance(VisionConstants.objectDetectionRotationTolerance);
        rotationController.enableContinuousInput(-180, 180);
    }


    public RobotCentric driveToObject(RobotCentric drive) {
        if (!canDetect() || Math.abs(bias) < 0.1) {
            return blindDrive(drive);
        }
        if (avgArea > 1.2) {
            return targetDrive(drive);
        }
        double rot = rotationController.calculate(bias, 0);
        return drive
            .withVelocityX(2 / (1 + Math.abs(rot / 5)))
            .withVelocityY(0)
            .withRotationalRate(rot);
    }
    public RobotCentric targetDrive(RobotCentric drive) {
        return drive
            .withVelocityX(1)
            .withVelocityY(0)
            .withRotationalRate(0);
    }
    public RobotCentric blindDrive(RobotCentric drive) {
        return drive
            .withVelocityX(1)
            .withVelocityY(0)
            .withRotationalRate(-Math.PI/3);
    }
    public RobotCentric zeroDrive(RobotCentric drive) {
        return drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
    }

    public Command objectTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        return drivetrain.applyRequest(() -> driveToObject(drive))
            .beforeStarting(() -> state = true)
            .finallyDo(interrupted -> state = false);
    }

    public Command autonObjectTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        return objectTrackCommand(drivetrain, drive).until(this::objectDone);
    }

    public Command autonObjectDetect(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        return autonObjectTrackCommand(drivetrain, drive)
            .andThen(drivetrain.applyRequest(() -> zeroDrive(drive)));
    }

    public boolean isTracking() {
        return state;
    }

    public double getBias() {
        return bias;
    }

    public boolean rotFinished() {
        return rotationController.atSetpoint();
    }

    public boolean canDetect()
    {
        if (camera == null || !camera.isConnected()) {
            return false;
        }

        if (camera.getLatestResult().getTargets().isEmpty())
        {
            return false;
        }

        return true;
    }

    public boolean objectDone() {
        return !canDetect();
    }

    @Override
    public void periodic() {

        if (camera == null || !camera.isConnected()) {
            SmartDashboard.putBoolean("CAMERAS: ArducamColor: Connected", false);
            bias = 0;
            // softBias = 0;
            return;
        }
        
        camera.checkForResult();

        if (!canDetect())
            return;

        SmartDashboard.putBoolean("CAMERAS: ArducamColor: Connected", true);
        List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();

        bias = 0.0;
        avgArea = 0;

        if (!targets.isEmpty()) {
            for (PhotonTrackedTarget target : targets) {

                double area = target.getArea();
                double yaw = target.getYaw();

                if (area <= 0 || Double.isNaN(area) || Double.isNaN(yaw)) {
                    continue;
                }

                double distanceFactor = 1.0 + area * 0.02;
                distanceFactor = Math.max(0.8, Math.min(1.2, distanceFactor));
                avgArea += area;

                bias += Math.signum(yaw)
                        * (area * VisionConstants.areaWeight
                        + Math.abs(yaw) * VisionConstants.yawWeight)
                        * distanceFactor;
            }
            bias /= targets.size();
            avgArea /= targets.size();
        }

        // if (softBias == 0.0) {
        //     softBias = bias;
        // }
        // softBias = (softBias * 0.9) + (bias * 0.1);

        // if (bias == 0.0) {
        //     softBias = 0.0;
        // }

        if (RobotContainer.debugMode)
        {
            SmartDashboard.putNumber("CAMERAS: ArducamColor: Object Bias", bias);
            SmartDashboard.putNumber("CAMERAS: ArducamColor: Soft Bias", softBias);
            SmartDashboard.putBoolean("CAMERAS: ArducamColor: Object Tracking Active", state);
            SmartDashboard.putNumber("CAMERAS: ArducamColor: Objects Detected", targets.size());
        }
    }
}
