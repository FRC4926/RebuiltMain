package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final int shooter1CanId = 9;
    public static final int shooter2CanId = 10;
    public static final int feederCanID = 11;
    public static final int hoodCanID = 12;
    
    public static Slot0Configs shooterPIDConfig = new Slot0Configs()
	    .withKP(0)
	    .withKI(0)
	    .withKD(0)
	    .withKS(0)
	    .withKV(0);
	    
    public static Slot0Configs hoodPIDConfig = new Slot0Configs()
	    .withKP(0)
	    .withKI(0)
	    .withKD(0)
	    .withKS(0)
	    .withKV(0);

    public static final double shooterCurrentLimit = 60;
    public static final double feederCurrentLimit = 30;
    public static final double hoodCurrentLimit = 20;

    public static final double hoodGearRatio = 1;

    public static final double feederEffort = 0.75;

    public static final double TOFConvergingPercent = 0.2;

    //distance (meters) to time of flight (s)
    public static final InterpolatingDoubleTreeMap distanceToTOF = new InterpolatingDoubleTreeMap();
    static 
    {
	    distanceToTOF.put(0.0, 0.0);
        distanceToTOF.put(1.0, 30.0);

    }

   //distance (meters) to angle table
    public static final InterpolatingDoubleTreeMap distanceToAngle1 = new InterpolatingDoubleTreeMap();
    static 
    {
	    distanceToAngle1.put(0.0, 0.0);
        distanceToAngle1.put(1.0, 30.0);
    }

    public static final InterpolatingDoubleTreeMap distanceToAngle2 = new InterpolatingDoubleTreeMap();
    static 
    {
	    distanceToAngle2.put(1.0, 30.0);
        distanceToAngle2.put(1.75, 52.5);
    }

    public static final InterpolatingDoubleTreeMap distanceToAngle3 = new InterpolatingDoubleTreeMap();
    static 
    {
	    distanceToAngle3.put(1.75, 52.5);
	    distanceToAngle3.put(2.75, 82.5);
    }

    //three RPM ranges each with their own angle LUT
    public static final InterpolatingDoubleTreeMap[] angleLookupTables = {distanceToAngle1, distanceToAngle2, distanceToAngle3};
    public static final double[] RPMRanges = {2000, 3000, 4500};
    public static final double[] rangeDistanceThresholds = {0.75, 1.5, 2.25};
    public static final double distanceThres = 0.08;

    public static final double idleSpeedRPM = 1000.0;
}
