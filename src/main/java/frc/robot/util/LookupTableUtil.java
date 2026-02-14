package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;

public class LookupTableUtil {

    private Pose2d effectiveHubPose = new Pose2d();
    private double distanceToHub = 0.0;
    private int currentRange = 0;

    public LookupTableUtil()
    {
        SmartDashboard.putNumber("Sim distance", 0.0);
    }

    public double getHoodAngle()
    {

        return ShooterConstants.angleLookupTables[currentRange].get(distanceToHub);
    }

    public double getTargetRPM()
    {
        return ShooterConstants.RPMRanges[currentRange];
    }

    public double distanceToHub(Pose2d robotPose)
    {
        return distanceBetween(effectiveHubPose, robotPose);
    }

    public Pose2d getUnmodifiedHubPose() {
        Pose2d unmodifiedHub = FieldConstants.hubCenterBlue;
        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)){
            unmodifiedHub = FieldConstants.hubCenterRed;
        }
        return unmodifiedHub;
    }

    public void updateEffectiveHub() {
        Translation2d hubShifts = new Translation2d();
        Pose2d currentHubPose = getUnmodifiedHubPose();

        Pose2d robotPose2d = RobotContainer.drivetrain.getState().Pose;
        double vx = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
        double vy = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;

        double distance = distanceBetween(robotPose2d, currentHubPose);
        double time = ShooterConstants.distanceToTOF.get(distance);

        //TODO manually end this if it never converges
        while (true)
        {
            hubShifts = new Translation2d(getUnmodifiedHubPose().getX()-vx*time, getUnmodifiedHubPose().getY()-vy*time);
            currentHubPose = getUnmodifiedHubPose().plus(new Transform2d(hubShifts, new Rotation2d()));
            distance = distanceBetween(currentHubPose, robotPose2d);
            double newTime = ShooterConstants.distanceToTOF.get(distance);

            if (withinConvergingTolerance(newTime, time))
            {
                time = newTime;
                break;
            } else
            {
                time = newTime;
            }
        }

        hubShifts = new Translation2d(getUnmodifiedHubPose().getX()-vx*time, getUnmodifiedHubPose().getY()-vy*time);
        effectiveHubPose = getUnmodifiedHubPose().plus(new Transform2d(hubShifts, new Rotation2d()));
        // distanceToHub = SmartDashboard.getNumber("Sim distance", 0.0);
        distanceToHub = distanceToHub(RobotContainer.drivetrain.getState().Pose);
    }

    public void updateCurrentRange()
    {
        if (distanceToHub > (ShooterConstants.rangeDistanceThresholds[currentRange] + ShooterConstants.distanceThres))
        {
            if (currentRange < ShooterConstants.rangeDistanceThresholds.length - 1)
            {
                currentRange++;
            }
        } else if (currentRange <= 0)
        {
            return;
        } else if (distanceToHub < (ShooterConstants.rangeDistanceThresholds[currentRange - 1] - ShooterConstants.distanceThres))
        {
            currentRange--;
        }
    }
    
    public Pose2d getEffectiveHubPose()
    {
        return effectiveHubPose;
    }

    public double distanceBetween(Pose2d a, Pose2d b){
        return Math.sqrt(Math.pow(a.getX()-b.getX(), 2)+Math.pow(a.getY()-b.getY(), 2));
    }

    public boolean withinConvergingTolerance(double a, double b)
    {
     return Math.abs(a - b) <= ShooterConstants.TOFConvergingPercent * Math.max(Math.abs(a), Math.abs(b));
    }

    public int getCurrentRange()
    {
        return currentRange;
    }
}
