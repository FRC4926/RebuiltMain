package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
   
    public LookupTableUtil lookupTableUtil = new LookupTableUtil();

    private LoggerUtil logger = new LoggerUtil("Shooter Subsystem", true);

    private boolean manualShot = false;

    private double snapPIDCalc = 0.0;

    public ShooterSubsystem() {

        // SmartDashboard.putNumber("Target RPM", 0); //4000
        // SmartDashboard.putNumber("Target Angle", 0);

        shooterMotor1.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        shooterMotor2.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.CounterClockwise_Positive)
        );

        feederMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        hoodMotor.getConfigurator().apply(
            new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive)
        );

        CurrentLimitsConfigs shooterCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ShooterConstants.shooterStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ShooterConstants.shooterSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs feederCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ShooterConstants.feederStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ShooterConstants.feederSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);

        CurrentLimitsConfigs hoodCurrentLimitsConfigs = new CurrentLimitsConfigs()
                .withStatorCurrentLimit(ShooterConstants.hoodStatorCurrentLimit)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(ShooterConstants.hoodSupplyCurrentLimit)
                .withSupplyCurrentLimitEnable(true);


        shooterMotor1.getConfigurator().apply(shooterCurrentLimitsConfigs);
        shooterMotor2.getConfigurator().apply(shooterCurrentLimitsConfigs);
        feederMotor.getConfigurator().apply(feederCurrentLimitsConfigs);
        hoodMotor.getConfigurator().apply(hoodCurrentLimitsConfigs);

        SoftwareLimitSwitchConfigs softLimitConf = new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitEnable(false)
            .withReverseSoftLimitThreshold(hoodMotorRotationsToDegrees(0))
            .withForwardSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(hoodMotorRotationsToDegrees(41.2));
        
        
        hoodMotor.getConfigurator().apply(softLimitConf);

        shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
        shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
        feederMotor.setNeutralMode(NeutralModeValue.Brake);
        hoodMotor.setNeutralMode(NeutralModeValue.Brake);

        shooterMotor1.getConfigurator().apply(ShooterConstants.shooterPIDConfig);
        shooterMotor2.getConfigurator().apply(ShooterConstants.shooterPIDConfig);

        hoodMotor.getConfigurator().apply(ShooterConstants.hoodPIDConfig);

        // shooterMotor2.setControl(new Follower(ShooterConstants.shooterRightCanId, MotorAlignmentValue.Opposed));
        hoodMotor.setPosition(0);

        ParentDevice.resetSignalFrequenciesForAll(shooterMotor1, shooterMotor2);
        ParentDevice.optimizeBusUtilizationForAll(feederMotor, hoodMotor);

        setNormalPIDValue();
    }

    public void shooterIdle(){
        shooterMotor1.set(ShooterConstants.idleShootEffort);
        shooterMotor2.set(ShooterConstants.idleShootEffort);

        feederMotor.setControl(new DutyCycleOut(ShooterConstants.idleFeedEffort));
    }

    public void setShooterRPMManual(double RPM){
        if (RPM < 150)
        {
            shooterMotor1.set(0.0);
            shooterMotor2.set(0.0);
            return;
        }
        shooterMotor1.setControl(new VelocityVoltage(RPM/60.0).withSlot(0));
        shooterMotor2.setControl(new VelocityVoltage(RPM/60.0).withSlot(0));

    }    
    
    public void calcRotRate() {
        SwerveDriveState state = RobotContainer.drivetrain.getState();
        Pose2d effectiveHubPose = lookupTableUtil.getEffectiveHubPose();
        double desiredX = effectiveHubPose.getX();
        double desiredY = effectiveHubPose.getY();
        double currentX = state.Pose.getX();
        double currentY = state.Pose.getY();

        double currentAngle = state.Pose.getRotation().getRadians();
        double angle = Math.atan2(desiredY - currentY, desiredX - currentX);
        snapPIDCalc = DriveConstants.snapToHubPID.calculate(currentAngle, angle);
    }

    public double getRotRate()
    {
        return snapPIDCalc;
    }

    public double getFeedRotRate() {
        double currentAngle =  RobotContainer.drivetrain.getState().Pose.getRotation().getRadians();
        double angle = Math.PI;

        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red))
            angle = 0.0;
            
        double rotRate = DriveConstants.snapToHubPID.calculate(currentAngle, angle);
        return rotRate;
    }

    public double getOffset() {
        double offset = -0.0;
        if (DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red)){
            offset = 0.0;
        }
        return offset;
    }

    public void updateShooter() {
        setShooterRPMManual(lookupTableUtil.getTargetRPM());
        setHoodAngleDegrees(lookupTableUtil.getHoodAngle());
        feederMotor.setControl(new DutyCycleOut(ShooterConstants.feederEffort));
    }

    public void updateFeeder()
    {
        feederMotor.setControl(new DutyCycleOut(ShooterConstants.feederEffort));
    }

    public void updateAngle() {
        if (manualShot)
            return;
        setHoodAngleDegrees(lookupTableUtil.getHoodAngle());
    }

    public void updateRPM() {
        setShooterRPMManual(lookupTableUtil.getTargetRPM());
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

    //supply current
    public double getShooterMotor1SupplyCurrent() {
        return shooterMotor1.getSupplyCurrent().getValueAsDouble();
    }
    public double getShooterMotor2SupplyCurrent() {
        return shooterMotor2.getSupplyCurrent().getValueAsDouble();
    }

    public double getFeederMotorSupplyCurrent() {
        return feederMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getHoodMotorSupplyCurrent() {
        return hoodMotor.getSupplyCurrent().getValueAsDouble();
    }

    public double getTotalShooterSupplyCurrent()
    {
        return getShooterMotor1SupplyCurrent() + getShooterMotor2SupplyCurrent() + getFeederMotorSupplyCurrent() + getHoodMotorSupplyCurrent();
    }

    public double getHoodMotorVoltage() {
        return hoodMotor.getClosedLoopOutput().getValueAsDouble();
    }


    public double getRPMError()
    {
        return Math.abs(lookupTableUtil.getTargetRPM() - getShooterAverageRPM());
    }

    public double getAngleError()
    {
        return Math.abs(lookupTableUtil.getHoodAngle() - getHoodAngleDegrees());
    }

    public boolean canShoot()
    {
        updateRPM();
        setFeedEffort(-0.2);
        return getRPMError() < ShooterConstants.RPMTolerance && getAngleError() < ShooterConstants.angleTolerance;
    }

    public void setFeedEffort(double effort)
    {
        feederMotor.set(effort);
    }

    public void shoot()
    {  
        updateRPM();
        setFeedEffort(ShooterConstants.feederEffort);
    }

    public Command shootCommand()
    {
        return (run(this::shoot));
    }

    public Command canShootCommand()
    {
        return Commands.idle().until(() -> canShoot());
    }

    public Command canShootManualCommand()
    {
        return Commands.idle().until(() -> canShootManual());
    }


    public void unJamShooter()
    {
        shooterMotor1.setControl(new DutyCycleOut(-0.2));
        shooterMotor2.setControl(new DutyCycleOut(-0.2));
        feederMotor.setControl(new DutyCycleOut(-0.6));
    }

    public Command unJamShooterCommand() {
        return runOnce(this:: unJamShooter);
    }

    public void manualShot()
    {
        manualShot = true;
        setHoodAngleDegrees(ShooterConstants.manualAngle);
        setShooterRPMManual(ShooterConstants.manualRPM);
        // setHoodAngleDegrees(lookupTableUtil.getHoodAngle());
        // setShooterRPMManual(lookupTableUtil.getTargetRPM());
        setFeedEffort(ShooterConstants.feederEffort);
    }

    public boolean canShootManual()
    {
        manualShot = true;
        setHoodAngleDegrees(ShooterConstants.manualAngle);
        setShooterRPMManual(ShooterConstants.manualRPM);
        // setHoodAngleDegrees(lookupTableUtil.getHoodAngle());
        // setShooterRPMManual(lookupTableUtil.getTargetRPM());
        return Math.abs(ShooterConstants.manualRPM - getShooterAverageRPM()) < ShooterConstants.RPMTolerance && Math.abs(ShooterConstants.manualAngle - getHoodAngleDegrees()) < ShooterConstants.angleTolerance;
        // return Math.abs(lookupTableUtil.getTargetRPM() - getShooterAverageRPM()) < ShooterConstants.RPMTolerance && Math.abs(lookupTableUtil.getHoodAngle() - getHoodAngleDegrees()) < ShooterConstants.angleTolerance;

    }

    public Command setHighPIDValue(){
        return Commands.parallel(
            new InstantCommand(() -> shooterMotor1.getConfigurator().apply(ShooterConstants.shooterPIDConfig2)),
            new InstantCommand(() -> shooterMotor2.getConfigurator().apply(ShooterConstants.shooterPIDConfig2))
        );
    }

    public Command setNormalPIDValue(){
        return Commands.parallel(
            new InstantCommand(() -> shooterMotor1.getConfigurator().apply(ShooterConstants.shooterPIDConfig)),
            new InstantCommand(() -> shooterMotor2.getConfigurator().apply(ShooterConstants.shooterPIDConfig))
        );
        
    }
    
    public Command manualShotCommand()
    {
        return run(this::manualShot);
    }

    public void setManual(boolean val)
    {
        manualShot = val;
    }

    @Override
    public void periodic() {
        lookupTableUtil.updateEffectiveDistance();
        lookupTableUtil.updateCurrentRange();
        calcRotRate();

        // double targetRPM = SmartDashboard.getNumber("Target RPM", 0); //40000.0

        // setShooterRPMManual(targetRPM);
        // setHoodAngleDegrees(SmartDashboard.getNumber("Target Angle", 0)); //17.2

        updateAngle();

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

        logger.put("Shooter Avg RPM", getShooterAverageRPM());
        logger.put("Shooter 1 RPM", getShooter1RPM());
        logger.put("Shooter 2 RPM", getShooter2RPM());

        logger.put("Feeder RPM", feederMotor.getVelocity().getValueAsDouble()*60.0);


        logger.put("RPM Error", (getRPMError()));

        logger.put("Shooter 1 Stator Current", getShooterMotor1StatorCurrent());
        logger.put("Shooter 2 Stator Current", getShooterMotor2StatorCurrent());
        logger.put("Feed Stator Current", getFeedMotorStatorCurrent());

        logger.put("Hood Angle (deg)", getHoodAngleDegrees(), true);
        logger.put("Hood Stator Current", getHoodMotorStatorCurrent());
        logger.put("Hood Voltage", getHoodMotorVoltage());


        logger.put("Commanded Hood Angle", lookupTableUtil.getHoodAngle());
        logger.put("Commanded RPM", lookupTableUtil.getTargetRPM());
        logger.put("Current Range", lookupTableUtil.getCurrentRange(), true);

        logger.put("Hub", lookupTableUtil.getUnmodifiedHubPose());
        logger.put("DISTANCE", lookupTableUtil.getDistanceToHub(), true);
        logger.put("Offset", lookupTableUtil.getOffset(), true);


    }

}
