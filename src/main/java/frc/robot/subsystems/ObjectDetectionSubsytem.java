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
    private final PIDController driveController = VisionConstants.objectDetectionDrivePIDController;
    private final PIDController rotationController = VisionConstants.objectDetectionRotationPIDController;

    private CameraWrapper camera = null;
    private double pitch = 0;
    private double bias = 0;

    PhotonTrackedTarget targetFuel = null;
    private double fuelNumber = 0;

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


    public RobotCentric driveToObject(CommandSwerveDrivetrain drivetrain, RobotCentric drive)     {
        if (getBias() != 0)
        {
            double rotCalculation = rotationController.calculate(getBias(), 0);
            return drive
                .withVelocityX(0)
                .withRotationalRate(rotCalculation);
        } else
        {
            return drive
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0);
        }
    }

    public RobotCentric zeroDrive(RobotCentric drive) {
       return drive
            .withRotationalRate(0)
            .withVelocityX(0)
            .withVelocityY(0);

    }
    public double getBias() {
        return bias;
    }
    public boolean rotFinished()
    {
        return rotationController.atSetpoint();
    }

    public Command objectTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
        return drivetrain.applyRequest(() -> driveToObject(drivetrain, drive));
    }

    public Command autonObjectTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
          return drivetrain.applyRequest(() -> driveToObject(drivetrain, drive)).until(this::objectDone);
    }

    public Command autonObjectDetect(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
        return autonObjectTrackCommand(drivetrain, drive)
            .andThen(new InstantCommand(() -> zeroDrive(drive)));
    }

    public boolean objectDone()
    {
        return targetFuel == null;
    }

    public boolean validObject(PhotonTrackedTarget object)
    {
        return true;
        //return object.getPitch() <= VisionConstants.objectDetectHorizonPitch;
    }

    @Override
    public void periodic() {
        if ((camera == null) || !camera.isConnected()) {
            SmartDashboard.putBoolean("COULD NOT CONNECT COLOR", true);
            return;
        }

        List<PhotonTrackedTarget> results = camera.getLatestResult().getTargets();
        
        bias = 0.0;

        for (PhotonTrackedTarget object: results)
        {
            bias += Math.signum(object.getYaw())*(object.getArea()*VisionConstants.areaWeight + Math.abs(object.getYaw())*VisionConstants.yawWeight)/results.size();
        }


        //TODO: look at edge cases like if there is no fuel

        SmartDashboard.putNumber("Bias", bias);
        SmartDashboard.putNumber("# of Fuel", results.size());
    }
}