package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX shooterMotor  = new TalonFX(ShooterConstants.shooterCanId);

    final VelocityVoltage RPS_request = new VelocityVoltage(0).withSlot(0);


    public ShooterSubsystem() {

        shooterMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        Slot0Configs slot0Conf = new Slot0Configs()
            .withKP(ShooterConstants.kP)
            .withKI(ShooterConstants.kI)
            .withKD(ShooterConstants.kD)
            .withKS(ShooterConstants.kS)
            .withKV(ShooterConstants.kS);


        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(30);
        shooterMotor.getConfigurator().apply(currentLimitsConfigs);

        shooterMotor.setNeutralMode(NeutralModeValue.Coast);

        shooterMotor.getConfigurator().apply(slot0Conf);
    }

    public double getShooterRPM()
    {
        return shooterMotor.getVelocity().getValueAsDouble()*60.0;
    }

    public void setReferenceVelocity() {
        shooterMotor.setControl(RPS_request.withVelocity(ShooterConstants.targetRPS));
    }

    public double getVelocity() {
        return shooterMotor.getVelocity().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return shooterMotor.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Shooter RPM", getShooterRPM());
    }

}
