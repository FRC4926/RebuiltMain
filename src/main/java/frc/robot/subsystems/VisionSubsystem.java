package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.VisionSystemSim;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
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
    boolean toggle = true;

    private final VisionSystemSim visionSim;


    public VisionSubsystem() {
        updateOrigin();

        for (VisionConstants.CameraWrapperConstants camConstant : VisionConstants.camConstants) {
            addCamera(camConstant.name(), camConstant.robotToCamera(), camConstant.trustFactor());
        }

        if (Robot.isSimulation()) {
            // visionSim = null;
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(FieldConstants.tagLayout);
            for (CameraWrapper cam : camWrappers) {
                cam.addToSimulator(visionSim);
            }
        } else {
            visionSim = null;
        }
    }

    public void updateOrigin() {
        // Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);
        FieldConstants.tagLayout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
    }

    public void addCamera(String camName, Transform3d robotToCam, double trustFactor) {
        camWrappers.add(new CameraWrapper(camName, robotToCam, FieldConstants.tagLayout, true, trustFactor, RobotContainer.drivetrain::addVisionMeasurement));
    }

    public List<CameraWrapper> getCameras()
    {
        return camWrappers;
    }

    // public EstimatedRobotPose[] getEstimatedGlobalPoses() {
    //     EstimatedRobotPose[] ret = new EstimatedRobotPose[camWrappers.size()];
    //     for (int i = 0; i < camWrappers.size(); i++) {
    //         Optional<EstimatedRobotPose> estimated = camWrappers.get(i).getEstimatedGlobalPose();
    //         ret[i] = estimated.isPresent() ? estimated.get() : null;
    //     }

    //     return ret;
    // }

    // public double[] getStandardDeviations()
    // {
    //     double[] ret = new double[camWrappers.size()];
    //     for (int i = 0; i < camWrappers.size(); i++) {
    //         if (camWrappers.get(i).isConnected())
    //             ret[i] = camWrappers.get(i).getStandardDeviation();
    //         else
    //             ret[i] = Double.POSITIVE_INFINITY;
    //     }

    //     return ret;
    // }

    public boolean poseIsValid(Pose2d pose) {
        return FieldConstants.fieldRect.contains(pose.getTranslation());
    }

    public void addVisionMeasurements(CommandSwerveDrivetrain drivetrain) {      
        for (CameraWrapper cam : camWrappers) {
            cam.updateResults();
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
        if (Robot.isSimulation()) {
            visionSim.update(RobotContainer.drivetrain.getState().Pose);
        } else {
            SmartDashboard.putString("Alliance", DriverStation.getAlliance().get().toString());
        }
    } 
}
