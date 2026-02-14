package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    public final TalonFX intakeMotor1  = new TalonFX(IntakeConstants.intake1CanId);
    public final TalonFX intakeMotor2  = new TalonFX(IntakeConstants.intake2CanId);
    public final TalonFX pivotMotor  = new TalonFX(IntakeConstants.pivotCanId);

    public IntakeSubsystem() {

        intakeMotor1.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
         intakeMotor2.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
         pivotMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        Slot0Configs slot0Conf = IntakeConstants.pivotPIDSlot0Configs;


        CurrentLimitsConfigs intakeCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(IntakeConstants.intakeCurrentLimit);
        intakeMotor1.getConfigurator().apply(intakeCurrentLimitsConfigs);
        intakeMotor2.getConfigurator().apply(intakeCurrentLimitsConfigs);

        CurrentLimitsConfigs pivotCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(IntakeConstants.pivotCurrentLimit);
        pivotMotor.getConfigurator().apply(pivotCurrentLimitsConfigs);

        intakeMotor1.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor2.setNeutralMode(NeutralModeValue.Coast);
        pivotMotor.setNeutralMode(NeutralModeValue.Brake);

        intakeMotor2.setControl(new Follower(IntakeConstants.intake2CanId, MotorAlignmentValue.Opposed));

        pivotMotor.getConfigurator().apply(slot0Conf);
        pivotMotor.setPosition(0);
    }

    public double getIntake1RPM()
    {
        return intakeMotor1.getVelocity().getValueAsDouble()*60.0;
    }

    public double getIntake2RPM()
    {
        return intakeMotor1.getVelocity().getValueAsDouble()*60.0;
    }

    public void setReferenceVelocity() {
        intakeMotor1.setControl(new DutyCycleOut(IntakeConstants.IntakeMotorEffort));
    }
    public Command pivotStaticDownCommand(){
        return runOnce(this::setPivotDownPosition);
    }
    public Command pivotStaticUpCommand(){
        return runOnce(this::setPivotUpPosition);
    }
    public Command pivotZeroCommand(){
        return runOnce(this::setPivotZero);
    }
    public Command intakeRunCommand(){
        return runOnce(this::setReferenceVelocity);
    }
    public Command zeroIntake() {
        return runOnce(this::zeroVelocity);
    }
    public void zeroVelocity(){
        intakeMotor1.setControl(new DutyCycleOut(0));
    }
    public void setPivotDownPosition() {
        pivotMotor.setControl(new PositionVoltage(rotationsFromDegrees(IntakeConstants.pivotDownPosition)).withSlot(0));
    }
    public void setPivotUpPosition() {
        pivotMotor.setControl(new PositionVoltage(rotationsFromDegrees(IntakeConstants.pivotUpPosition)).withSlot(0));
    }
    public void setPivotZero(){
        pivotMotor.setControl(new DutyCycleOut(0));
    }

    private double rotationsFromDegrees(double degrees)
    {
        return degrees*IntakeConstants.gearRatio/360.0;
    }

    private double degreesFromRotations(double motorRot)
    {
        return motorRot/IntakeConstants.gearRatio*360.0;
    }


    public void setPivotVelocity(double velocity) {
        pivotMotor.setControl(new VelocityVoltage(velocity));
    }

    public double getIntake1Velocity() {
        return intakeMotor1.getVelocity().getValueAsDouble();
    }

    public double getStatorIntake1Current() {
        return intakeMotor1.getStatorCurrent().getValueAsDouble();
    }
    public double getIntake2Velocity() {
        return intakeMotor1.getVelocity().getValueAsDouble();
    }
    public double getIntakeAverageRPM(){
        return (getIntake1RPM()+getIntake2RPM())/2;
    }

    public double getStatorIntake2Current() {
        return intakeMotor1.getStatorCurrent().getValueAsDouble();
    }
    public double getPivotVelocity() {
        return intakeMotor1.getVelocity().getValueAsDouble();
    }

    public double getPivotCurrent() {
        return intakeMotor1.getStatorCurrent().getValueAsDouble();
    }
    public double getPivotUpAngle(){
        return IntakeConstants.pivotUpPosition;
    }
    public double getPivotDownAngle(){
        return IntakeConstants.pivotDownPosition;
    }
    public double getPivotAngle(){
        return degreesFromRotations(pivotMotor.getPosition().getValueAsDouble());
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Intake 1 RPM", getIntake1RPM());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Intake 2 RPM", getIntake2RPM());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Intake Average RPM", getIntakeAverageRPM());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Pivot Angle", getPivotAngle());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Pivot Up Angle", getPivotUpAngle());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Pivot Down Angle", getPivotDownAngle());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Intake 1 Stator Current", getStatorIntake1Current());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Intake 2 Stator Current", getStatorIntake2Current());
        SmartDashboard.putNumber("INTAKE SUBSYSTEM: Pivot Stator Current", getPivotCurrent());
        
    }

}
