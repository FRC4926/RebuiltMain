package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.LoggerUtil;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

public class IntakeSubsystem extends SubsystemBase {
    public final TalonFX intakeMotor1  = new TalonFX(IntakeConstants.intake1CanId); //intakeRight
    public final TalonFX intakeMotor2  = new TalonFX(IntakeConstants.intake2CanId); //intakeLeft
    public final TalonFX pivotMotor  = new TalonFX(IntakeConstants.pivotCanId);
    
    private LoggerUtil logger = new LoggerUtil("Intake Subsystem");

    public IntakeSubsystem() {
        // SmartDashboard.putNumber("Target Pivot Angle", 0.0);

        intakeMotor1.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
         intakeMotor2.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );
         pivotMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        CurrentLimitsConfigs intakeCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(IntakeConstants.intakeCurrentLimit);
        intakeMotor1.getConfigurator().apply(intakeCurrentLimitsConfigs);
        intakeMotor2.getConfigurator().apply(intakeCurrentLimitsConfigs);

        CurrentLimitsConfigs pivotCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(IntakeConstants.pivotCurrentLimit);
        pivotMotor.getConfigurator().apply(pivotCurrentLimitsConfigs);

        intakeMotor1.setNeutralMode(NeutralModeValue.Coast);
        intakeMotor2.setNeutralMode(NeutralModeValue.Coast);

        // pivotMotor.getConfigurator().apply(new FeedbackConfigs().withRotorToSensorRatio(IntakeConstants.gearRatio));
        pivotMotor.setNeutralMode(NeutralModeValue.Coast);

        pivotMotor.getConfigurator().apply(IntakeConstants.pivotDownPIDSlot0Configs);
        pivotMotor.getConfigurator().apply(IntakeConstants.pivotUpPIDSlot1Configs);
        pivotMotor.getConfigurator().apply(IntakeConstants.pivotOscillateConfigs);
        pivotMotor.setPosition(0);

        intakeMotor1.getConfigurator().apply(IntakeConstants.intakePIDSlot0Configs);
        // intakeMotor2.getConfigurator().apply(IntakeConstants.intakePIDSlot0Configs);

        intakeMotor2.setControl(new Follower(IntakeConstants.intake1CanId, MotorAlignmentValue.Opposed));

        ParentDevice.resetSignalFrequenciesForAll(intakeMotor1, intakeMotor2);
        ParentDevice.optimizeBusUtilizationForAll(pivotMotor);
    }

    public double getIntake1RPM()
    {
        return intakeMotor1.getVelocity().getValueAsDouble()*60.0;
    }

    public double getIntake2RPM()
    {
        return intakeMotor2.getVelocity().getValueAsDouble()*60.0;
    }

    public void setReferenceVelocity() {
        intakeMotor1.setControl(new DutyCycleOut(IntakeConstants.intakeMotorEffort));
        // intakeMotor2.setControl(new DutyCycleOut(IntakeConstants.intakeMotorEffort));
    }

    public void intakeRun() {
        intakeMotor1.setControl(IntakeConstants.intakeRunControl);
        // intakeMotor2.setControl(IntakeConstants.intakeRunControl);
    }

    public void setReferenceVelocity(double effort) {
        intakeMotor1.setControl(new DutyCycleOut(effort));
        // intakeMotor2.setControl(new DutyCycleOut(effort));
    }

    public Command oscillatePivotCommand()
    {
        return Commands.sequence(pivotOscillateCommand(IntakeConstants.pivotOscillateUpPosition), 
            new WaitCommand(IntakeConstants.pivotOscillateBetween), 
            pivotOscillateCommand(IntakeConstants.pivotOscillateDownPosition), 
            new WaitCommand(IntakeConstants.pivotOscillateBetween)).repeatedly();
    }

    public Command pivotDownCommand(){
        return runOnce(this::setPivotDownPosition);
    }
    public Command pivotUpCommand(){
        return runOnce(this::setPivotUpPosition);
    }

    //TODO make it add requirements
    public Command pivotStaticManualCommand(double angle){
        return new InstantCommand(() -> setPivotPosition(angle));
    }

    //TODO make it add requirements
     public Command pivotOscillateCommand(double angle){
        return new InstantCommand(() -> setOscillatePosition(angle));
    }
    public Command pivotZeroCommand(){
        return runOnce(this::setPivotZero);
    }
    public Command intakeRunCommand(){
        return runOnce(this::intakeRun);
    }
    public Command zeroIntake() {
        return runOnce(this::zeroVelocity);
    }
    public void zeroVelocity(){
        intakeMotor1.setControl(new DutyCycleOut(0));
        // intakeMotor2.setControl(new DutyCycleOut(0));

    }
    public void setPivotDownPosition() {
        pivotMotor.setControl(new PositionVoltage(rotationsFromDegrees(IntakeConstants.pivotDownPosition)).withSlot(0));
    }
    public void setPivotUpPosition() {
        pivotMotor.setControl(new PositionVoltage(rotationsFromDegrees(IntakeConstants.pivotUpPosition)).withSlot(1));
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
  
    public double getIntake2Velocity() {
        return intakeMotor2.getVelocity().getValueAsDouble();
    }
    public double getIntakeAverageRPM(){
        return (getIntake1RPM()+getIntake2RPM())/2;
    }

    public double getStatorIntake1Current() {
        return intakeMotor1.getStatorCurrent().getValueAsDouble();
    }

    public double getStatorIntake2Current() {
        return intakeMotor2.getStatorCurrent().getValueAsDouble();
    }

    public double getSupplyIntake1Current() {
        return intakeMotor1.getSupplyCurrent().getValueAsDouble();
    }

    public double getSupplyIntake2Current() {
        return intakeMotor2.getSupplyCurrent().getValueAsDouble();
    }
    public double getPivotVelocity() {
        return pivotMotor.getVelocity().getValueAsDouble();
    }

    public double getPivotCurrent() {
        return pivotMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getPivotAngle(){
        return degreesFromRotations(pivotMotor.getPosition().getValueAsDouble());
    }

    public void setPivotPosition(double angle) {
        int slot = (getPivotAngle() > angle) ? 1 : 0;
        SmartDashboard.putNumber("Intake Slot", slot);
        pivotMotor.setControl(new PositionVoltage(rotationsFromDegrees(angle)).withSlot(slot));
    }

    public void setOscillatePosition(double angle) {
        pivotMotor.setControl(new PositionVoltage(rotationsFromDegrees(angle)).withSlot(2));
    }

    @Override
    public void periodic() {
        // setPivotPosition(SmartDashboard.getNumber("Target Pivot Angle", 0.0));

        logger.put("Intake 1 RPM", getIntake1RPM());
        logger.put("Intake 2 RPM", getIntake2RPM());
        logger.put("Intake Average RPM", getIntakeAverageRPM());
        logger.put("Pivot Angle", getPivotAngle());
        logger.put("Pivot Angle Actual", pivotMotor.getPosition().getValueAsDouble());
        logger.put("Intake 1 Stator Current", getStatorIntake1Current());
        logger.put("Intake 2 Stator Current", getStatorIntake2Current());
        // logger.put("Intake 1 Supply Current", getStatorIntake1Current());
        // logger.put("Intake 2 Supply Current", getStatorIntake2Current());
        logger.put("Pivot Stator Current", getPivotCurrent());
    }
}
