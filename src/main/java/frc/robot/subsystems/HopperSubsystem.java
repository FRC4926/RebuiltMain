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

public class HopperSubsystem extends SubsystemBase {
    public final TalonFX hopperMotorLeft  = new TalonFX(HopperConstants.hopperMotorLeftID);
    public final TalonFX hopperMotorCenter  = new TalonFX(HopperConstants.hopperMotorCenterID);
    public final TalonFX hopperMotorRight = new TalonFX(HopperConstants.hopperMotorRightID);

    final VelocityVoltage RPS_request = new VelocityVoltage(0).withSlot(0);


    public HopperSubsystem() {

        hopperMotorLeft.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
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

    public void highEffort() {
        hopperMotorLeft.setControl(new DutyCycleOut(HopperConstants.highEffort));
        hopperMotorCenter.setControl(new DutyCycleOut(HopperConstants.highEffort));
        hopperMotorRight.setControl(new DutyCycleOut(HopperConstants.highEffort));
    }
    public void lowEffort() {
        hopperMotorLeft.setControl(new DutyCycleOut(HopperConstants.lowEffort));
        hopperMotorCenter.setControl(new DutyCycleOut(HopperConstants.lowEffort));
        hopperMotorRight.setControl(new DutyCycleOut(HopperConstants.lowEffort));
    }
    public void setVelocityZero(){
        hopperMotorLeft.setControl(new DutyCycleOut(0));
        hopperMotorCenter.setControl(new DutyCycleOut(0));
        hopperMotorRight.setControl(new DutyCycleOut(0));
    }
    public Command highEffortCommand(){
        return runOnce(this::highEffort);
    }
    public Command lowEffortCommand() {
        return runOnce(this::lowEffort);
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

        if (RobotContainer.debugMode)
        {
            SmartDashboard.putNumber("HOPPER SUBSYSTEM: Left RPM", getHopperLeftRPM());
            SmartDashboard.putNumber("HOPPER SUBSYSTEM: Center RPM", getHopperCenterRPM());
            SmartDashboard.putNumber("HOPPER SUBSYSTEM: Right RPM", getHopperRightRPM());

            SmartDashboard.putNumber("HOPPER SUBSYSTEM: Left Current",getStatorCurrentMotorLeft());
            SmartDashboard.putNumber("HOPPER SUBSYSTEM: Center Current",getStatorCurrentMotorCenter());
            SmartDashboard.putNumber("HOPPER SUBSYSTEM: Right Current",getStatorCurrentMotorRight());
        }
    }

}