package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimberConstants;
import frc.robot.util.LoggerUtil;

public class ClimberSubsystem extends SubsystemBase {

    public final TalonFX climberMotorLeft  = new TalonFX(ClimberConstants.climberMotorLeftID);
    public final TalonFX climberMotorRight = new TalonFX(ClimberConstants.climberMotorRightID);

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    private LoggerUtil logger = new LoggerUtil("Climber Subsystem");

    public ClimberSubsystem() {

        climberMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        climberMotorRight.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        climberMotorLeft.setNeutralMode(NeutralModeValue.Brake);
        climberMotorRight.setNeutralMode(NeutralModeValue.Brake);

        Slot0Configs slot0Configs = ClimberConstants.climberPIDConstants;
        climberMotorLeft.getConfigurator().apply(slot0Configs);

        CurrentLimitsConfigs currentLimits =
            new CurrentLimitsConfigs().withStatorCurrentLimit(ClimberConstants.climberCurrentLimit);

        climberMotorLeft.getConfigurator().apply(currentLimits);
        climberMotorRight.getConfigurator().apply(currentLimits);

        climberMotorRight.setControl(
            new Follower(ClimberConstants.climberMotorLeftID, MotorAlignmentValue.Opposed)
        );

        climberMotorLeft.setPosition(0);
    }

    public void setPercentOutput(double percent) {
        climberMotorLeft.setControl(new DutyCycleOut(percent));
    }

    public void setVelocity(double rotationsPerSecond) {
        climberMotorLeft.setControl(velocityRequest.withVelocity(rotationsPerSecond));
    }

    public void setPosition(double rotations) {
        climberMotorLeft.setControl(positionRequest.withPosition(rotations));
    }

    public void zeroPosition() {
        climberMotorLeft.setPosition(0);
    }

    public double getLeftRPM() {
        return climberMotorLeft.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getRightRPM() {
        return climberMotorRight.getVelocity().getValueAsDouble() * 60.0;
    }

    public double getAverageRPM() {
        return (getLeftRPM() + getRightRPM()) / 2.0;
    }

    public double getPositionRotations() {
        return climberMotorLeft.getPosition().getValueAsDouble();
    }

    public double getStatorCurrent() {
        return climberMotorLeft.getStatorCurrent().getValueAsDouble();
    }

    public Command runPercentCommand(double percent) {
        return run(() -> setPercentOutput(percent)).finallyDo(interrupted -> setPercentOutput(0));
    }

    public Command moveToPositionCommand(double rotations) {
        return runOnce(() -> setPosition(rotations));
    }

    public Command zeroCommand() {
        return runOnce(this::zeroPosition);
    }

    @Override
    public void periodic() {
        logger.put("Left RPM", getLeftRPM());
        logger.put("Right RPM", getRightRPM());
        logger.put("Average RPM", getAverageRPM());
        logger.put("Position Rotations", getPositionRotations());
        logger.put("Stator Current", getStatorCurrent());
    }
}