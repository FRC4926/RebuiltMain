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
import frc.robot.subsystems.ShooterSubsystem;

public class LookupTableUtil {

    private Pose2d effectiveHubPose = new Pose2d();
    private double distanceToHub = 0.0;
    private double distanceToFeed = 0.0;
    private int currentRange = 0;
    private double offset = 0.6;

    public LookupTableUtil()
    {
        // SmartDashboard.putNumber("Sim distance", 0.0);
        SmartDashboard.putNumber("Manual Hood Angle", 0);
        SmartDashboard.putNumber("Manual RPM", 0);
    }

    public double getHoodAngle()
    {
        double angle = 0.0;
        if (inAllianceZone())
            angle = ShooterConstants.angleLookupTables[currentRange].get(distanceToHub);
        else
            angle = ShooterConstants.feedAngle;
        return SmartDashboard.getNumber("Manual Hood Angle", 0);
        // return angle;
    }

    public double getTargetRPM()
    {
        return SmartDashboard.getNumber("Manual RPM", 0);
        // if (inAllianceZone())
        //     return ShooterConstants.RPMRanges[currentRange];
        // else
        //     return ShooterConstants.feedRPM;
    }

    public double distanceToHub(Pose2d robotPose)
    {
        return distanceBetween(effectiveHubPose, robotPose);
    }

    public double distanceToFeed(Pose2d robotPose)
    {
        return Math.abs(getFeedLine() - robotPose.getX());
    }

    public boolean inAllianceZone()
    {
        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)){
            return RobotContainer.drivetrain.getState().Pose.getX() > (FieldConstants.fieldLength - FieldConstants.allianceZoneLineBlue);
        } else
        {
            return RobotContainer.drivetrain.getState().Pose.getX() < FieldConstants.allianceZoneLineBlue;
        }
    
    }

    public double getDistanceToHub()
    {
        return distanceToHub;
    }

    public Pose2d getUnmodifiedHubPose() {
        Pose2d unmodifiedHub = FieldConstants.hubCenterBlue;
        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)){
            unmodifiedHub = FieldConstants.hubCenterRed;
        }
        return unmodifiedHub;
    }

    public double getFeedLine() {
        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)){
            return FieldConstants.fieldLength - FieldConstants.allianceZoneLineBlue;
        } else
        {
            return FieldConstants.feedLineBlue;
        }
    }

    public void updateEffectiveDistance() {
        // Translation2d hubShifts = new Translation2d();
        // Pose2d currentHubPose = getUnmodifiedHubPose();

        // Pose2d robotPose2d = RobotContainer.drivetrain.getState().Pose;
        // double vx = RobotContainer.drivetrain.getState().Speeds.vxMetersPerSecond;
        // double vy = RobotContainer.drivetrain.getState().Speeds.vyMetersPerSecond;

        // double distance = distanceBetween(robotPose2d, currentHubPose);
        // double time = ShooterConstants.distanceToTOF.get(distance);

        // //10 loops before manual break
        // for(int i = 0; i < 10; i++)
        // {
        //     hubShifts = new Translation2d(getUnmodifiedHubPose().getX()-vx*time, getUnmodifiedHubPose().getY()-vy*time);
        //     currentHubPose = getUnmodifiedHubPose().plus(new Transform2d(hubShifts, new Rotation2d()));
        //     distance = distanceBetween(currentHubPose, robotPose2d);
        //     double newTime = ShooterConstants.distanceToTOF.get(distance);

        //     if (withinConvergingTolerance(newTime, time))
        //     {
        //         time = newTime;
        //         break;
        //     } else
        //     {
        //         time = newTime;
        //     }
        // }

        // hubShifts = new Translation2d(getUnmodifiedHubPose().getX()-vx*time, getUnmodifiedHubPose().getY()-vy*time);
        // effectiveHubPose = getUnmodifiedHubPose().plus(new Transform2d(hubShifts, new Rotation2d()));

        effectiveHubPose = getUnmodifiedHubPose();
        // distanceToHub = SmartDashboard.getNumber("Sim distance", 0.0);
        distanceToHub = distanceToHub(RobotContainer.drivetrain.getState().Pose) + getOffset();
        distanceToFeed = distanceToFeed(RobotContainer.drivetrain.getState().Pose);
    }

    public double getOffset()
    {
        return offset;
    }

    public void incrementOffset()
    {
        offset += 0.05;
    }

    public void decrementOffset()
    {
        offset -= 0.05;
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
