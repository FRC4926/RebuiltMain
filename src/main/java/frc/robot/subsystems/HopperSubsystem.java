package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.HopperConstants;
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


        CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(HopperConstants.hopperCurrentLimit);
        hopperMotorLeft.getConfigurator().apply(currentLimitsConfigs);
        hopperMotorLeft.setNeutralMode(NeutralModeValue.Coast);

        hopperMotorCenter.getConfigurator().apply(currentLimitsConfigs);
        hopperMotorCenter.setNeutralMode(NeutralModeValue.Coast);

        hopperMotorRight.getConfigurator().apply(currentLimitsConfigs);
        hopperMotorRight.setNeutralMode(NeutralModeValue.Coast);
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

    public Command zeroVelocity(){
        return runOnce(this::setVelocityZero);
    }

    public double getStatorCurrentMotorLeft() {
        return hopperMotorLeft.getStatorCurrent().getValueAsDouble();
    }

    public double getStatorCurrentMotorCenter() {
        return hopperMotorCenter.getStatorCurrent().getValueAsDouble();
    }

    public double getStatorCurrentMotorRight() {
        return hopperMotorRight.getStatorCurrent().getValueAsDouble();
    }

    @Override
    public void periodic() {
        // if (RobotContainer.driverController.a().getAsBoolean()) {
        //     hopperMotorCenter.set(1);
        //     hopperMotorLeft.set(-0.3);
        //     hopperMotorRight.set(0.3);
        // } else if (RobotContainer.driverController.x().getAsBoolean()) {
        //     hopperMotorLeft.set(0);
        //     hopperMotorRight.set(0);
        //     hopperMotorCenter.set(-1);

        // } else {
        //     hopperMotorCenter.set(0);
        //     hopperMotorLeft.set(0);
        //     hopperMotorRight.set(0);
        // }


        // if (RobotContainer.debugMode)
        // {
        //     SmartDashboard.putNumber("HOPPER SUBSYSTEM: Left RPM", getHopperLeftRPM());
        //     SmartDashboard.putNumber("HOPPER SUBSYSTEM: Center RPM", getHopperCenterRPM());
        //     SmartDashboard.putNumber("HOPPER SUBSYSTEM: Right RPM", getHopperRightRPM());
        //     SmartDashboard.putNumber("HOPPER SUBSYSTEM: Left Current", getStatorCurrentMotorLeft());
        //     SmartDashboard.putNumber("HOPPER SUBSYSTEM: Center Current", getStatorCurrentMotorCenter());
        //     SmartDashboard.putNumber("HOPPER SUBSYSTEM: Right Current", getStatorCurrentMotorRight());
        // }

        // logger.put("Left RPM", getHopperLeftRPM());
        // logger.put("Center RPM", getHopperCenterRPM());
        // logger.put("Right RPM", getHopperRightRPM());

        // logger.put("Left Current", getStatorCurrentMotorLeft());
        // logger.put("Center Current", getStatorCurrentMotorCenter());
        // logger.put("Right Current", getStatorCurrentMotorRight());
    }

}