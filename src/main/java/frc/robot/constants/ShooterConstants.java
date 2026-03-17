package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public class ShooterConstants {
    public static final int shooterRightCanId = 9;
    public static final int shooterLeftCanId = 10;
    public static final int feederCanID = 11;
    public static final int hoodCanID = 12;
    
    public static Slot0Configs shooterPIDConfig = new Slot0Configs()
	    .withKP(0.538)
	    .withKI(0)
	    .withKD(0)
	    .withKS(.19)
	    .withKV(0.1225);
	    
    public static Slot0Configs hoodPIDConfig = new Slot0Configs()
	    .withKP(40)
	    .withKI(1.5)
	    .withKD(0)
	    .withKG(0.4);

    public static final double shooterCurrentLimit = 60;
    public static final double feederCurrentLimit = 70;
    public static final double hoodCurrentLimit = 40;

    public static final double hoodGearRatio = 185.0/18.0;

    //Hood angle range: 14  to 420
    public static final double feederEffort = 1;

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
	    distanceToAngle1.put(0.9144, 4.67);
        distanceToAngle1.put(1.38, 8.5);
        distanceToAngle1.put(1.73, 10.0);
        distanceToAngle1.put(2.05, 12.11);
        distanceToAngle1.put(2.44, 13.56);
        distanceToAngle1.put(2.83, 16.16);
    }

    public static final InterpolatingDoubleTreeMap distanceToAngle2 = new InterpolatingDoubleTreeMap();
    static 
    {
	    distanceToAngle2.put(2.83, 13.56);
        distanceToAngle2.put(3.31, 14.85);
        distanceToAngle2.put(3.6, 16.31);
        distanceToAngle2.put(3.87, 16.96);
        distanceToAngle2.put(4.18, 16.96);
    }

    public static final InterpolatingDoubleTreeMap distanceToAngle3 = new InterpolatingDoubleTreeMap();
    static 
    {
		distanceToAngle2.put(4.18, 18.0);
        distanceToAngle2.put(4.49, 19.5);
        distanceToAngle2.put(4.83, 19.5);
        distanceToAngle2.put(5.05, 21.0);
    }


    //RPM ranges each with their own angle LUT
    public static final InterpolatingDoubleTreeMap[] angleLookupTables = {distanceToAngle1, distanceToAngle2, distanceToAngle3};
    public static final double[] RPMRanges = {3800, 4200, 4500};
    public static final double[] rangeDistanceThresholds = {2.83, 4.18, 5.5};
    public static final double distanceThres = 0.1;

    public static final double idleSpeedRPM = 0.0;

    public static final double RPMTolerance = 75;
}
