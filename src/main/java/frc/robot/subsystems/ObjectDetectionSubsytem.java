package frc.robot.subsystems;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
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

    public ObjectDetectionSubsytem() {

        for (var cam : RobotContainer.visionSubsystem.getCameras()) {
            if (cam.getCamera().getName().equals("ArducamColor")) {
                camera = cam;
                break;
            }
        }

        driveController.setTolerance(0);
        rotationController.setTolerance(VisionConstants.objectDetectionRotationTolerance);
        rotationController.enableContinuousInput(-180, 180);
    }

    public RobotCentric driveToObject(RobotCentric drive) {

        boolean hasTarget =
            !camera.getCamera().getLatestResult().getTargets().isEmpty();

        if (hasTarget && Math.abs(bias) > 0.1) {
            double rot = rotationController.calculate(bias, 0);

            return drive
                .withVelocityX(2 / (1 + Math.abs(rot / 5)))
                .withVelocityY(0)
                .withRotationalRate(rot);
        }

        return zeroDrive(drive);
    }

    public RobotCentric zeroDrive(RobotCentric drive) {
        return drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
    }

    public Command objectTrackCommand(
            CommandSwerveDrivetrain drivetrain,
            RobotCentric drive) {

        return drivetrain.applyRequest(() -> driveToObject(drive))
            .beforeStarting(() -> state = true)
            .finallyDo(interrupted -> state = false);
    }

    public Command autonObjectTrackCommand(
            CommandSwerveDrivetrain drivetrain,
            RobotCentric drive) {

        return objectTrackCommand(drivetrain, drive)
            .until(this::objectDone);
    }

    public Command autonObjectDetect(
            CommandSwerveDrivetrain drivetrain,
            RobotCentric drive) {

        return autonObjectTrackCommand(drivetrain, drive)
            .andThen(new InstantCommand(() -> drivetrain.applyRequest(() -> zeroDrive(drive))));
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

    public boolean objectDone() {
        if (camera == null || !camera.isConnected()) {
            return true;
        }
        return camera.getLatestResult().getTargets().isEmpty();
    }

    @Override
    public void periodic() {

        if (camera == null || !camera.isConnected()) {
            SmartDashboard.putBoolean("Color Camera Connected", false);
            bias = 0;
            softBias = 0;
            return;
        }

        SmartDashboard.putBoolean("Color Camera Connected", true);

        List<PhotonTrackedTarget> targets =
                camera.getLatestResult().getTargets();

        bias = 0.0;

        if (!targets.isEmpty()) {
            for (PhotonTrackedTarget target : targets) {

                double area = target.getArea();
                double yaw = target.getYaw();

                if (area <= 0 || Double.isNaN(area) || Double.isNaN(yaw)) {
                    continue;
                }

                double distanceFactor = 1.0 + area * 0.02;
                distanceFactor = Math.max(0.8, Math.min(1.2, distanceFactor));

                bias += Math.signum(yaw)
                        * (area * VisionConstants.areaWeight
                        + Math.abs(yaw) * VisionConstants.yawWeight)
                        * distanceFactor;
            }
            bias /= targets.size();
        }

        if (softBias == 0.0) {
            softBias = bias;
        }
        softBias = (softBias * 0.9) + (bias * 0.1);

        if (bias == 0.0) {
            softBias = 0.0;
        }

        SmartDashboard.putNumber("Object Bias", bias);
        SmartDashboard.putNumber("Soft Bias", softBias);
        SmartDashboard.putBoolean("Object Tracking Active", state);
        SmartDashboard.putNumber("Objects Detected", targets.size());
    }
}
