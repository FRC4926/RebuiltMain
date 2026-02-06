package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.util.CameraWrapper;

public class VisionSubsystem extends SubsystemBase {
   private List<CameraWrapper> camWrappers = new ArrayList<>();
 
    Optional<EstimatedRobotPose> estimatedPose;

    public VisionSubsystem() {
        updateOrigin();

        for (VisionConstants.CameraWrapperConstants camConstant : VisionConstants.camConstants) {
            addCamera(camConstant.name(), camConstant.robotToCamera(), camConstant.trustFactor());
        }
    }

    public void updateOrigin() {
        // Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        FieldConstants.tagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    public void addCamera(String camName, Transform3d robotToCam, double trustFactor) {
        camWrappers.add(new CameraWrapper(camName, robotToCam, FieldConstants.tagLayout, true, trustFactor));
    }

    public List<CameraWrapper> getCameras()
    {
        return camWrappers;
    }

    public EstimatedRobotPose[] getEstimatedGlobalPoses() {
        EstimatedRobotPose[] ret = new EstimatedRobotPose[camWrappers.size()];
        for (int i = 0; i < camWrappers.size(); i++) {
            Optional<EstimatedRobotPose> estimated = camWrappers.get(i).getEstimatedGlobalPose();
            ret[i] = estimated.isPresent() ? estimated.get() : null;
        }

        return ret;
    }

    public double[] getStandardDeviations()
    {
        SmartDashboard.putNumber("Here1", camWrappers.size());
        double[] ret = new double[camWrappers.size()];
        for (int i = 0; i < camWrappers.size(); i++) {
            if (camWrappers.get(i).isConnected())
                ret[i] = camWrappers.get(i).getStandardDeviation();
            else
                ret[i] = 9999999.0;
        }

        return ret;
    }

    public boolean poseIsValid(EstimatedRobotPose pose) {
        return FieldConstants.fieldRect.contains(pose.estimatedPose.getTranslation().toTranslation2d());
    }

    public void addVisionMeasurements(CommandSwerveDrivetrain drivetrain) {
        SmartDashboard.putNumber("Added vision measurement", SmartDashboard.getNumber("Added vision measurement", 0) + 1);
        EstimatedRobotPose[] poses = getEstimatedGlobalPoses();
        double[] standardDeviations = getStandardDeviations();

        for (int i = 0; i < poses.length; i++) {
            if (poses[i] != null && poseIsValid(poses[i]))
                drivetrain.addVisionMeasurement(
                    poses[i].estimatedPose.toPose2d(),
                    Utils.fpgaToCurrentTime(poses[i].timestampSeconds),
                    new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {
                        standardDeviations[i], // x
                        standardDeviations[i], // y
                        VisionConstants.kalmanRotationStdDev  // rotation
                    })
                );
        }
    }
    
    public Command addVisionMeasurementsOnceCommand(CommandSwerveDrivetrain drivetrain) {
        return runOnce(() -> addVisionMeasurements(drivetrain)).ignoringDisable(true);
    }
    public Command addVisionMeasurementsCommand(CommandSwerveDrivetrain drivetrain) {
        return run(() -> addVisionMeasurements(drivetrain)).ignoringDisable(true);
    }

    @Override
    public void periodic() 
    {
        
        for (CameraWrapper cam : camWrappers) {
            cam.checkForResult();
        }
        SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());

        // if (Robot.isSimulation()) {
        //     visionSim.update(RobotContainer.drivetrain.getState().Pose);
        // }
        
    } 
}
