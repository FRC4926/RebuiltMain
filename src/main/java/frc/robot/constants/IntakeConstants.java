package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;

public class IntakeConstants {
    public static final int intake1CanId = 14;
    public static final int intake2CanId = 15;
    public static final int pivotCanId = 13;
    
    public static final Slot0Configs pivotPIDSlot0Configs = new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKS(0)
            .withKV(0);

    public static final double targetRPS = 3000.0/60.0;
    public static final double IntakeMotorEffort = 0.75;
    public static final double pivotUpPosition = 0;
    public static final double gearRatio = 0;
    public static final double pivotDownPosition = 90;
}
