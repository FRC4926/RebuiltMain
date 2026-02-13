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

    public static final double idleSpeedRPM = 1000.0;
    public static final double feederEffort = 0.75;
    public static InterpolatingDoubleTreeMap distanceToTOF = new InterpolatingDoubleTreeMap();
    public static InterpolatingDoubleTreeMap distanceToAngleTable = new InterpolatingDoubleTreeMap();

    public static final double rpmLow = 0;
    public static final double rpmHigh = 0;
    public static final double thresholdDistance = 0;

}
