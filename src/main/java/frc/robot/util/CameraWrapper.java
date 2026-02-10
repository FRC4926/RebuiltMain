package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.constants.VisionConstants;

public class CameraWrapper {
    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;
    private final PhotonPoseEstimator poseEstimator;
    private final StructPublisher<Pose2d> posePublisher;
    private Transform3d robotToCam;
    private PhotonPipelineResult latestResult;
    private List<PhotonPipelineResult> unreadResults;
    private boolean publishPose;
    private double trustFactor;
    private final EstimateConsumer estConsumer;
    private Pose2d currentPose = new Pose2d();

    public CameraWrapper(String camName, Transform3d _robotToCam, AprilTagFieldLayout fieldLayout, boolean _publishPose, double _trustFactor, EstimateConsumer addVisionConsumer) {
        camera = new PhotonCamera(camName);
        trustFactor = _trustFactor;

        robotToCam = _robotToCam;
        poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
        
        estConsumer = addVisionConsumer;

        publishPose = _publishPose;
        if (publishPose) {
            posePublisher = NetworkTableInstance.getDefault().getStructTopic(camName + " pose", Pose2d.struct).publish();
        } else {
            posePublisher = null;
        }
   
        if (Robot.isSimulation()) {
            SimCameraProperties camProps = new SimCameraProperties();
            // A 640 x 480 camera with a 100 degree diagonal FOV.
            camProps.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            camProps.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            camProps.setFPS(45);
            // The average and standard deviation in milliseconds of image data latency.
            camProps.setAvgLatencyMs(25);
            camProps.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, camProps);
        } else {
            cameraSim = null;
        }
    }

    public void addToSimulator(VisionSystemSim sim) {
        if (Robot.isSimulation()) {
            sim.addCamera(cameraSim, robotToCam.inverse());
        }
    }

    public Transform3d getRobotToCam() {
        return robotToCam;
    }

    public String getName()
    {
        return camera.getName();
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    public double getTrustFactor()
    {
        return trustFactor;
    }

    public void checkForResult() {
        if (!camera.isConnected())
        {
            unreadResults = new ArrayList<>();
            return;
        }

        unreadResults = camera.getAllUnreadResults();
        if (!unreadResults.isEmpty())
            latestResult = unreadResults.get(unreadResults.size() - 1);
    }

    public void updateResults() {
        checkForResult();
        addEstimatedGlobalPose();
    }

    public PhotonPipelineResult getLatestResult() {
        return latestResult;
    }

    public boolean targetIsValid(PhotonTrackedTarget target) {
        // SmartDashboard.putNumber(getName() +" Pose Ambiguity", target.getPoseAmbiguity());
        if (target.getPoseAmbiguity() > VisionConstants.maximumAmbiguity)
            return false;

        return true;
    }

    public void addEstimatedGlobalPose() { 
        currentPose = new Pose2d();
        if (!camera.isConnected() || unreadResults.size() <= 0)
        {
            SmartDashboard.putBoolean(getName(), false);
        } else
        {
            SmartDashboard.putBoolean(getName(), true);
        }

    
        for (var result : unreadResults) {
            Optional<EstimatedRobotPose> currentEst = poseEstimator.estimateCoprocMultiTagPose(result);
            if (currentEst.isEmpty()) {
                currentEst = poseEstimator.estimateLowestAmbiguityPose(result);
            }

            currentEst.ifPresent(
                est -> {
                    // Change our trust in the measurement based on the tags we can see
                    var translationDeviation = getEstimatedStandardDeviation(result.getTargets());

                    Matrix<N3, N1> estStdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[] {
                        translationDeviation, // x
                        translationDeviation, // y
                        VisionConstants.kalmanRotationStdDev  // rotation
                    });

                    currentPose = est.estimatedPose.toPose2d();

                    if (RobotContainer.visionSubsystem.poseIsValid(currentPose))
                    {
                        estConsumer.accept(currentPose, est.timestampSeconds, estStdDevs);
                    }
                });
        }

        if (publishPose) {
            posePublisher.set(currentPose);

        }

        //SmartDashboard.putBoolean("Pose estimator is present for" + camera.getName(), estimated.isPresent());

    }

    public double getID()
    {
        if (latestResult != null)
        {
            var bestTarget = latestResult.getBestTarget();
            return latestResult.hasTargets() ? bestTarget.getFiducialId() : -1;
        } else
        {
            return -1;
        }
    }

    public PhotonTrackedTarget getBestTarget()
    {
        return latestResult.getBestTarget();
    }

    public PhotonCamera getCamera()
    {
        return camera;
    }

    public double getEstimatedStandardDeviation(List<PhotonTrackedTarget> targets)
    {
        double totalDistance = 0;
        double totalTags = 0;
        for (var tag : targets) {
            if (!targetIsValid(tag))
                continue;
            totalDistance += tag.getBestCameraToTarget().getTranslation().getDistance(new Translation3d());
            totalTags++;
        }


        double avgDistance = totalDistance/totalTags;
        double stdDev = VisionConstants.kalmanPositionStdDevCoeefficient
            * Math.pow(avgDistance, 2)
            / totalTags
            * trustFactor;
        SmartDashboard.putNumber(camera.getName() + " stddev", stdDev);

        if (totalTags <= 0)
        {
            return Double.POSITIVE_INFINITY;
        }
        return stdDev;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}