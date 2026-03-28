package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.HopperConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.LoggerUtil;

public class HopperSubsystem extends SubsystemBase {
    public final TalonFX hopperMotorLeft  = new TalonFX(HopperConstants.hopperMotorLeftID);
    public final TalonFX hopperMotorCenter  = new TalonFX(HopperConstants.hopperMotorCenterID);
    public final TalonFX hopperMotorRight = new TalonFX(HopperConstants.hopperMotorRightID);

    final VelocityVoltage RPS_request = new VelocityVoltage(0).withSlot(0);

    private LoggerUtil logger = new LoggerUtil("Hopper Subsystem");

    public HopperSubsystem() {

        hopperMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        );
        hopperMotorCenter.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
        hopperMotorRight.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(HopperConstants.hopperStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(HopperConstants.hopperSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);

            
        hopperMotorLeft.getConfigurator().apply(currentLimitsConfigs);
        hopperMotorLeft.setNeutralMode(NeutralModeValue.Coast);

        hopperMotorCenter.getConfigurator().apply(currentLimitsConfigs);
        hopperMotorCenter.setNeutralMode(NeutralModeValue.Coast);

        hopperMotorRight.getConfigurator().apply(currentLimitsConfigs);
        hopperMotorRight.setNeutralMode(NeutralModeValue.Coast);

        ParentDevice.optimizeBusUtilizationForAll(hopperMotorLeft, hopperMotorCenter, hopperMotorRight);

    }

    public double getHopperLeftRPM()
    {
        return hopperMotorLeft.getVelocity().getValueAsDouble()*60.0;
    }
    public double getHopperCenterRPM()
    {
        return hopperMotorCenter.getVelocity().getValueAsDouble()*60.0;
    }
    public double getHopperRightRPM()
    {
        return hopperMotorRight.getVelocity().getValueAsDouble()*60.0;
    }

    public void positiveEffort() {
        hopperMotorLeft.setControl(new DutyCycleOut(HopperConstants.sideEffort));
        hopperMotorCenter.setControl(new DutyCycleOut(HopperConstants.centerEffort));
        hopperMotorRight.setControl(new DutyCycleOut(HopperConstants.sideEffort));
    }

    public void negativeEffort() {
        hopperMotorLeft.setControl(new DutyCycleOut(HopperConstants.sideEffort));
        hopperMotorCenter.setControl(new DutyCycleOut(-HopperConstants.centerEffort));
        hopperMotorRight.setControl(new DutyCycleOut(HopperConstants.sideEffort));
    }
    public void setVelocityZero(){
        hopperMotorLeft.setControl(new DutyCycleOut(0));
        hopperMotorCenter.setControl(new DutyCycleOut(0));
        hopperMotorRight.setControl(new DutyCycleOut(0));
    }

    public Command positiveEffortCommand() {
        return runOnce(this::positiveEffort);
    }

    public Command negativeEffortCommand() {
        return runOnce(this::negativeEffort);
    }

    public Command zeroEffortCommand(){
        return runOnce(this::setVelocityZero);
    }

    public double getLeftStatorCurrent() {
        return hopperMotorLeft.getStatorCurrent().getValueAsDouble();
    }

    public double getCenterStatorCurrent() {
        return hopperMotorCenter.getStatorCurrent().getValueAsDouble();
    }

    public double getRightStatorCurrent() {
        return hopperMotorRight.getStatorCurrent().getValueAsDouble();
    }

    
    public double getLeftSupplyCurrent() {
        return hopperMotorLeft.getSupplyCurrent().getValueAsDouble();
    }

    public double getCenterSupplyCurrent() {
        return hopperMotorCenter.getSupplyCurrent().getValueAsDouble();
    }

    public double getRightSupplyCurrent() {
        return hopperMotorRight.getSupplyCurrent().getValueAsDouble();
    }

    public double getTotalHopperSupplyCureent()
    {
        return getLeftSupplyCurrent() + getCenterSupplyCurrent() + getRightSupplyCurrent();
    }

    @Override
    public void periodic() {
        logger.put("Left RPM", getHopperLeftRPM());
        logger.put("Center RPM", getHopperCenterRPM());
        logger.put("Right RPM", getHopperRightRPM());

        logger.put("Left Stator Current", getLeftStatorCurrent());
        logger.put("Center Stator Current", getCenterStatorCurrent());
        logger.put("Right Stator Current", getRightStatorCurrent());
    }

}