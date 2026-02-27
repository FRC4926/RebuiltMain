package frc.robot.subsystems;

import java.util.List;
import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.CameraWrapper;
import frc.robot.util.LoggerUtil;

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

    private Timer objectDectectionTime = new Timer();

    private LoggerUtil logger = new LoggerUtil("Object Detection");

    public ObjectDetectionSubsytem() {
        //I know its weird but surely its fine
        camera = new CameraWrapper("ArducamColor", new Transform3d(new Translation3d(11.804*0.0254, 0, 20*0.0254), new Rotation3d(0,-15*Math.PI/180,Math.PI/3)), FieldConstants.tagLayout, false, 0, RobotContainer.drivetrain::addVisionMeasurement);
        driveController.setTolerance(0);
        rotationController.setTolerance(VisionConstants.objectDetectionRotationTolerance);
        rotationController.enableContinuousInput(-180, 180);
    }

    boolean isPosePastMidline(Pose2d pose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue) {
            return pose.getMeasureX().magnitude() > FieldConstants.fieldLength/2;
        }
        return pose.getMeasureX().magnitude() < FieldConstants.fieldLength/2;
    }

    public double getTargetRotRate() {
        // Check for blind drive
        if (!canDetect() || Math.abs(bias) < VisionConstants.objectDetectMinBias) return -Math.PI/3;
        // Check for target drive
        if (avgArea > VisionConstants.objectDetectAreaThreshold) return 0;
        return rotationController.calculate(bias, 0);
    }

    public double getForwardSpeed(double rot) {
        // Check for blind drive
        if (!canDetect() || Math.abs(bias) < VisionConstants.objectDetectMinBias) return 1;
        // Check for target drive
        if (avgArea > VisionConstants.objectDetectAreaThreshold) return 1;
        return 2 / (1 + Math.abs(rot / 5));
    }

    public RobotCentric driveToObjectTeleOp(RobotCentric drive) {
        double rot = getTargetRotRate();
        double speed = getForwardSpeed(rot);
        return drive
            .withVelocityX(speed)
            .withVelocityY(0)
            .withRotationalRate(rot);
    }

    public RobotCentric driveToObjectAuton(RobotCentric drive) {
        double rot = getTargetRotRate();
        double speed = getForwardSpeed(rot);
        Pose2d initialPose = RobotContainer.drivetrain.getState().Pose;
        double newAngle = initialPose.getRotation().getRadians() + rot;
        Transform2d transform1s = new Transform2d(new Translation2d(speed * Math.cos(newAngle), speed * Math.sin(newAngle)), new Rotation2d(rot));
        if (isPosePastMidline(initialPose.plus(transform1s))) {
            return drive.withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(-Math.PI/3);
        }
        return drive
            .withVelocityX(speed)
            .withVelocityY(0)
            .withRotationalRate(rot);
    }

    public RobotCentric zeroDrive(RobotCentric drive) {
        return drive
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0);
    }

    public Command objectTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        return drivetrain.applyRequest(() -> driveToObjectTeleOp(drive))
            .beforeStarting(() -> state = true)
            .finallyDo(interrupted -> state = false);
    }

    // public Command autonObjectTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
    //     return objectTrackCommand(drivetrain, drive).until(this::objectDone);
    // }

    // public Command autonObjectDetect(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
    //     return autonObjectTrackCommand(drivetrain, drive)
    //         .andThen(drivetrain.applyRequest(() -> zeroDrive(drive)));
    // }

    public Command autonIntake(double totalTime, CommandSwerveDrivetrain drivetrain, RobotCentric drive, HopperSubsystem hopper, IntakeSubsystem intake) {
        Command startCmd = runOnce(objectDectectionTime::restart)
            .andThen(() -> state = true)
            .andThen(intake.intakeRunCommand())
            .andThen(hopper.highEffortCommand());
        Command followCmd = drivetrain.applyRequest(() -> driveToObjectAuton(drive));
        BooleanSupplier isDone = () -> objectDectectionTime.hasElapsed(totalTime);
        Command endCmd = drivetrain.applyRequestOnce(() -> zeroDrive(drive))
            .andThen(intake.zeroIntake())
            .andThen(hopper.lowEffortCommand())
            .andThen(() -> state = false);
        return startCmd.andThen(followCmd.until(isDone)).andThen(endCmd);
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
            logger.put("Connected", false);
            bias = 0;
            // softBias = 0;
            return;
        }
        
        camera.checkForResult();

        if (!canDetect())
            return;

        logger.put("Connected", true);
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

        // if (RobotContainer.debugMode)
        // {
        //     SmartDashboard.putNumber("CAMERAS: ArducamColor: Object Bias", bias);
        //     SmartDashboard.putNumber("CAMERAS: ArducamColor: Soft Bias", softBias);
        //     SmartDashboard.putBoolean("CAMERAS: ArducamColor: Object Tracking Active", state);
        //     SmartDashboard.putNumber("CAMERAS: ArducamColor: Objects Detected", targets.size());
        // }
        logger.put("Object Bias", bias);
        logger.put("Soft Bias", softBias);
        logger.put("Object Tracking Active", state);
        logger.put("Objects Detected", targets.size(), true);
    }
}
