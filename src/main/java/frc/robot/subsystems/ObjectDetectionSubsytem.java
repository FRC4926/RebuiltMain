package frc.robot.subsystems;

import java.util.List;
import java.util.function.IntSupplier;

import org.photonvision.simulation.VisionTargetSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.AutonConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.CameraWrapper;
import frc.robot.util.LimelightHelpers;


public class ObjectDetectionSubsytem extends SubsystemBase {
    private final PIDController driveController = VisionConstants.objectDetectionDrivePIDController;
    private final PIDController rotationController = VisionConstants.objectDetectionRotationPIDController;

    private CameraWrapper camera = null;
    private double yaw = 0;
    private double pitch = 0;

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
        if (targetFuel != null)
        {
            double rotMeasurement = getYaw();
            double rotSetpoint = 0;

            double rotCalculation = rotationController.calculate(rotMeasurement, rotSetpoint);


            return drive
                .withVelocityX(1.75)
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

    public double getYaw()
    {
        return yaw;
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
        return object.getPitch() <= VisionConstants.objectDetectHorizonPitch;
    }

    @Override
    public void periodic() {
        if ((camera == null) || !camera.isConnected()) {
            SmartDashboard.putBoolean("COULD NOT CONNECT COLOR", true);
            targetFuel = null;
            return;
        }

        List<PhotonTrackedTarget> results = camera.getLatestResult().getTargets();
        PhotonTrackedTarget bestTarget = null;

        for (int i = results.size() - 1; i >= 0; i--)
        {
            PhotonTrackedTarget currentObject = results.get(i);
            if (!validObject(currentObject))
            {
                results.remove(i);
                continue;
            }

            if (bestTarget == null)
            {
                bestTarget = currentObject;
            } else if (currentObject.getArea() > bestTarget.getArea())
            {
                bestTarget = currentObject;
            }
        }

        if (bestTarget != null) {
            targetFuel = bestTarget;
            yaw = targetFuel.getYaw();
            pitch = targetFuel.getPitch();
            fuelNumber = results.size();
        } 
        else {
           targetFuel = null;  
           fuelNumber = 0;
           yaw = 0;
           pitch = 0;   
        }

        SmartDashboard.putBoolean("Has Target", targetFuel != null);
        SmartDashboard.putNumber("Best object ID:", targetFuel.objDetectId); //TODO: test with pitch/yaw instead
        SmartDashboard.putNumber("# of Fuel", fuelNumber);
    }
}
