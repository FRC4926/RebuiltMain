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

    public static Slot0Configs shooterPIDConfig2 = new Slot0Configs()
	    .withKP(0.838)
	    .withKI(0)
	    .withKD(0)
	    .withKS(.19)
	    .withKV(0.125);
	    
    public static Slot0Configs hoodPIDConfig = new Slot0Configs()
	    .withKP(80)
	    .withKI(1.5)
	    .withKD(0)
	    .withKG(0.4);

    public static final double shooterStatorCurrentLimit = 60;
    public static final double shooterSupplyCurrentLimit = 60;

    public static final double feederStatorCurrentLimit = 60;
    public static final double feederSupplyCurrentLimit = 60;

    public static final double hoodStatorCurrentLimit = 30;
    public static final double hoodSupplyCurrentLimit = 30;


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
        distanceToAngle1.put(0.0, 8.0);
	    distanceToAngle1.put(1.4, 8.0);
        distanceToAngle1.put(1.75, 9.5);
        distanceToAngle1.put(2.063, 11.0);
        distanceToAngle1.put(2.34, 12.25);
        distanceToAngle1.put(2.623, 13.25);
        distanceToAngle1.put(2.937, 14.5);
        distanceToAngle1.put(3.24, 15.625);

        
    }

    public static final InterpolatingDoubleTreeMap distanceToAngle2 = new InterpolatingDoubleTreeMap();
    static 
    {
	    distanceToAngle2.put(2.937, 13.75);
        distanceToAngle2.put(3.24, 15.25);
        distanceToAngle2.put(3.57, 17.25);
        distanceToAngle2.put(3.825, 18.5);
        distanceToAngle2.put(4.11, 18.75);
    }

    public static final InterpolatingDoubleTreeMap distanceToAngle3 = new InterpolatingDoubleTreeMap();
    static 
    {
		distanceToAngle3.put(3.825, 17.25);
        distanceToAngle3.put(4.11, 17.75);
        distanceToAngle3.put(4.44, 18.5);
        distanceToAngle3.put(4.765, 20.25);
        distanceToAngle3.put(5.095, 24.0);
        distanceToAngle3.put(6.0, 24.0);

    }

    // public static final InterpolatingDoubleTreeMap distanceToAngleFeed = new InterpolatingDoubleTreeMap();
    // static 
    // {
	// 	distanceToAngleFeed.put(0.0, 0.0);
    //     distanceToAngleFeed.put(20.0, 28.0);
    // }


    //RPM ranges each with their own angle LUT
    public static final InterpolatingDoubleTreeMap[] angleLookupTables = {distanceToAngle1, distanceToAngle2, distanceToAngle3};
    public static final double[] RPMRanges = {3800, 4200, 4800}; //3850, 4350
    public static final double feedRPM = 4300;
    public static final double[] rangeDistanceThresholds = {2.937, 3.825, 6.0};
    public static final double distanceThres = 0.15;

    public static final double feedAngle = 22.5;

    public static final double idleShootEffort = 0.5;
    public static final double idleFeedEffort = -0.05;

    public static final double RPMTolerance = 80;
    public static final double angleTolerance = 0.5;

    public static final double timeTillOscillation = 2.0;

    //manual
    public static final double manualRPM = 4200;
    public static final double manualAngle = 18.0;
}
