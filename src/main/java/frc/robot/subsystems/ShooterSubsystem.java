package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.util.LoggerUtil;
import frc.robot.util.LookupTableUtil;

public class ShooterSubsystem extends SubsystemBase {
    public final TalonFX shooterMotor1  = new TalonFX(ShooterConstants.shooterRightCanId);
    public final TalonFX shooterMotor2  = new TalonFX(ShooterConstants.shooterLeftCanId);
    public final TalonFX feederMotor  = new TalonFX(ShooterConstants.feederCanID);
    public final TalonFX hoodMotor  = new TalonFX(ShooterConstants.hoodCanID);
   
    final VelocityVoltage idleSpeed = new VelocityVoltage(ShooterConstants.idleSpeedRPM/60.0).withSlot(0);

    LookupTableUtil lookupTableUtil = new LookupTableUtil();

    private LoggerUtil logger = new LoggerUtil("Shooter Subsystem");

    public ShooterSubsystem() {

        shooterMotor1.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        feederMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        hoodMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ShooterConstants.shooterCurrentLimit);
        CurrentLimitsConfigs feederCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ShooterConstants.feederCurrentLimit);
        CurrentLimitsConfigs hoodCurrentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(ShooterConstants.hoodCurrentLimit);


        shooterMotor1.getConfigurator().apply(shooterCurrentLimitsConfigs);
        shooterMotor2.getConfigurator().apply(shooterCurrentLimitsConfigs);
        feederMotor.getConfigurator().apply(feederCurrentLimitsConfigs);
        hoodMotor.getConfigurator().apply(hoodCurrentLimitsConfigs);

        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);
        hoodMotor.setNeutralMode(NeutralModeValue.Coast); //TODO: change to brake

        shooterMotor1.getConfigurator().apply(ShooterConstants.shooterPIDConfig);
        shooterMotor2.getConfigurator().apply(ShooterConstants.hoodPIDConfig);

        shooterMotor2.setControl(new Follower(ShooterConstants.shooterRightCanId, MotorAlignmentValue.Opposed));
        hoodMotor.setPosition(0);
    }
    
    public void shooterIdle(){
        shooterMotor1.setControl(idleSpeed);
        feederMotor.setControl(new DutyCycleOut(0));
        hoodMotor.setControl(new DutyCycleOut(0));
    }

    public void setShooterRPMManual(double RPM){
        shooterMotor1.setControl(new VelocityVoltage(RPM/60.0).withSlot(0));
    }    
    public double getRotRate() {
        SwerveDriveState state = RobotContainer.drivetrain.getState();
        Pose2d effectiveHubPose = lookupTableUtil.getEffectiveHubPose();
        double desiredX = effectiveHubPose.getX();
        double desiredY = effectiveHubPose.getY();
        double currentX = state.Pose.getX();
        double currentY = state.Pose.getY();

        double currentAngle = state.Pose.getRotation().getRadians();
        double angle = Math.atan2(desiredY - currentY, desiredX - currentX);
        double rotRate = DriveConstants.snapToHubPID.calculate(currentAngle, angle);
        return rotRate;
    }

    public void updateShooter() {
        shooterMotor1.setControl(new VelocityVoltage(lookupTableUtil.getTargetRPM()));
        setHoodAngleDegrees(lookupTableUtil.getHoodAngle());
        feederMotor.setControl(new DutyCycleOut(ShooterConstants.feederEffort));
    }

    public Command shooterIdleCommand() {
        return runOnce(this::shooterIdle);
    }

    public Command updateShooterCommand() {
        return run(this::updateShooter);
    }

    public Command manualRPMCommand(double RPM) {
        return runOnce(() -> setShooterRPMManual(RPM));
    }

    public double getShooter1RPM()
    {
        return shooterMotor1.getVelocity().getValueAsDouble()*60.0;
    }

     public double getShooter2RPM()
    {
        return shooterMotor2.getVelocity().getValueAsDouble()*60.0;
    }

    public double hoodMotorRotationsToDegrees(double rot) {
        return rot/ShooterConstants.hoodGearRatio*360.0;
    }

    public double degreesToHoodMotorRotations(double deg) {
        return deg/360*ShooterConstants.hoodGearRatio;
    }

    public double getHoodAngleDegrees() {
        return hoodMotorRotationsToDegrees(hoodMotor.getPosition().getValueAsDouble());
    }

    public void setHoodAngleDegrees(double deg) {
        hoodMotor.setControl(new PositionVoltage(degreesToHoodMotorRotations(deg)).withSlot(0));
    }

    public void setHoodEffort(double effort) {
        hoodMotor.set(effort);
    }

    public double getShooterAverageRPM()
    {
        return (shooterMotor1.getVelocity().getValueAsDouble()*60.0+shooterMotor2.getVelocity().getValueAsDouble()*60.0)/2;
    }

    public double getShooterMotor1StatorCurrent() {
        return shooterMotor1.getStatorCurrent().getValueAsDouble();
    }
    public double getShooterMotor2StatorCurrent() {
        return shooterMotor2.getStatorCurrent().getValueAsDouble();
    }

    public double getFeedMotorStatorCurrent() {
        return feederMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getHoodMotorStatorCurrent() {
        return hoodMotor.getStatorCurrent().getValueAsDouble();
    }

    public boolean shouldUpdateShooter()
    {
        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)){
            return RobotContainer.drivetrain.getState().Pose.getX() > (FieldConstants.fieldLength - FieldConstants.allianceZoneLine);
        } else
        {
            return RobotContainer.drivetrain.getState().Pose.getX() < FieldConstants.allianceZoneLine;
        }
    
    }

    @Override
    public void periodic() {
        lookupTableUtil.updateEffectiveHub();
        lookupTableUtil.updateCurrentRange();

        setHoodAngleDegrees(10);


        // if (RobotContainer.driverController.b().getAsBoolean() || RobotContainer.driverController.a().getAsBoolean() || RobotContainer.driverController.x().getAsBoolean()) {
        //     setShooterRPMManual(4000);
        // } else {
        //     setShooterRPMManual(0);
        // }
        // if (RobotContainer.driverController.a().getAsBoolean()) {
        //     feederMotor.setControl(new DutyCycleOut(ShooterConstants.feederEffort));
        // } else if (RobotContainer.driverController.x().getAsBoolean()) {
        //     feederMotor.setControl(new DutyCycleOut(-0.8));
        // } else {
        //     feederMotor.setControl(new DutyCycleOut(0));
        // }
        // setHoodAngleDegrees(320.0);

        // shooterMotor1.set(0.5);
        // feederMotor.set(0.5);

        // logger.put("Shooter Avg RPM", getShooterAverageRPM());
        // logger.put("Shooter 1 RPM", getShooter1RPM());
        // logger.put("Shooter 2 RPM", getShooter2RPM());
        // logger.put("Shooter 1 Stator Current", getShooterMotor1StatorCurrent());
        // logger.put("Shooter 2 Stator Current", getShooterMotor2StatorCurrent());

        // logger.put("Feed Stator Current", getFeedMotorStatorCurrent());

        // logger.put("Hood Angle (deg)", getHoodAngleDegrees());
        // logger.put("Hood Stator Current", getHoodMotorStatorCurrent());

        // logger.put("Commanded Hood Angle", lookupTableUtil.getHoodAngle());
        // logger.put("Commanded RPM", lookupTableUtil.getTargetRPM());
        // logger.put("Current Range", lookupTableUtil.getCurrentRange());
    }

}
