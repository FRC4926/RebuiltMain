package frc.robot.constants;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;

public class IntakeConstants {
    public static final int intake1CanId = 14; //intakeRight
    public static final int intake2CanId = 15; //intakeLeft
    public static final int pivotCanId = 13;

    public static final double intakeStatorCurrentLimit = 90;
    public static final double intakeSupplyCurrentLimit = 60;

    public static final double pivotStatorCurrentLimit = 30;
    public static final double pivotSupplyCurrentLimit = 30;

    
    public static final Slot0Configs pivotDownPIDSlot0Configs = new Slot0Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(2.5)
        .withKI(0)
        .withKD(0)
        .withKG(0.0035);

    public static final Slot1Configs pivotUpPIDSlot1Configs = new Slot1Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(1.75)
        .withKI(0)
        .withKD(0)
        .withKG(0.0035);

    public static final Slot2Configs pivotOscillateConfigs= new Slot2Configs()
        .withGravityType(GravityTypeValue.Arm_Cosine)
        .withKP(0.7)
        .withKI(0)
        .withKD(0)
        .withKG(0.0035);


    public static final Slot0Configs intakePIDSlot0Configs = new Slot0Configs()
        .withKP(0.425)
        .withKI(0)
        .withKD(0)
        .withKS(0)
        .withKV(0.108);

    public static final double targetRPS = 4000.0/60.0;
    public static final double intakeMotorEffort = 1;
    public static final double gearRatio = 62.1176470588; //(22/17)*48
    public static final double pivotUpPosition = 138;
    public static final double pivotDownPosition = -1;
    public static final double pivotOscillateUp1Position = 45;
    public static final double pivotOscillateUp2Position = 65;
    public static final double pivotOscillateDownPosition = 10;

    public static final double pivotOscillateBetweenTime = 0.5;

    public static final ControlRequest intakeRunControl = new VelocityVoltage(targetRPS).withSlot(0);
}
