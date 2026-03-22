// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.IntakeConstants;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ObjectDetectionSubsytem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentric relativeDrive = new SwerveRequest.RobotCentric()
        .withDeadband(DriveConstants.MaxSpeed * 0.1).withRotationalDeadband(DriveConstants.MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

            
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(DriveConstants.MaxSpeed); 

    public static final CommandXboxController driverController = new CommandXboxController(0);
    public static final CommandJoystick operatorController = new CommandJoystick(1);

    public static SendableChooser<Command> autonChooser;

    public static double translationRateLimit = 1.15;
    public SlewRateLimiter driveX = new SlewRateLimiter(translationRateLimit);
    public SlewRateLimiter driveY = new SlewRateLimiter(translationRateLimit);
    public SlewRateLimiter rot = new SlewRateLimiter(2);

    //subsystems
    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public static VisionSubsystem visionSubsystem;
    public static ObjectDetectionSubsytem detectionSubsystem;
    
    public static ShooterSubsystem shooterSubsystem;
    public static HopperSubsystem hopperSubsystem;
    public static IntakeSubsystem intakeSubsystem;

    public RobotContainer() 
    {
        shooterSubsystem = new ShooterSubsystem();
        hopperSubsystem = new HopperSubsystem();
        intakeSubsystem = new IntakeSubsystem();
        visionSubsystem = new VisionSubsystem();
        // detectionSubsystem = new ObjectDetectionSubsytem();

        // new EventTrigger("PointToHub").onTrue(drivetrain.overrideRot());
        // new EventTrigger("PointToHub").onFalse(drivetrain.clearOverride());

        // NamedCommands.registerCommand("AutoTrackCommand", detectionSubsystem.autonIntake(5, drivetrain, relativeDrive, hopperSubsystem, intakeSubsystem));
        // NamedCommands.registerCommand("AutoFaceHub", drivetrain.snapToHuAutonCommand(drive));
        // NamedCommands.registerCommand("ZeroDrive", new InstantCommand(() -> drivetrain.zeroDrive(relativeDrive)));

        NamedCommands.registerCommand("Shoot", autonShoot());
        NamedCommands.registerCommand("ClearIntake", intakeSubsystem.clearIntake());
        NamedCommands.registerCommand("RunIntake", intakeSubsystem.pivotDownCommand().andThen(intakeSubsystem.intakeRunCommand()));
        NamedCommands.registerCommand("StopIntake", intakeSubsystem.zeroIntake());
        NamedCommands.registerCommand("PivotUp", intakeSubsystem.pivotUpCommand());

        autonChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Autonomous", autonChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driveY.calculate(-driverController.getLeftY()) * DriveConstants.MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driveX.calculate(-driverController.getLeftX()) * DriveConstants.MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rot.calculate(driverController.getRightX()) * DriveConstants.MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // shooterSubsystem.setDefaultCommand(Commands.run(shooterSubsystem::shooterIdleCommand, shooterSubsystem));
        hopperSubsystem.setDefaultCommand(Commands.run(hopperSubsystem::zeroEffortCommand, hopperSubsystem));
        // intakeSubsystem.setDefaultCommand(intakeSubsystem.zeroIntake().andThen(intakeSubsystem.pivotZeroCommand()));
        visionSubsystem.setDefaultCommand(visionSubsystem.addVisionMeasurementsCommand(drivetrain));

        // Idle while the robot is disabled. This ensures the configured 6-7
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        
        driverController.leftTrigger().whileTrue(shoot());
        driverController.leftTrigger().onFalse(shooterDefault());
        

        
        driverController.rightTrigger().onTrue(intakeSubsystem.intakeRunCommand().andThen(intakeSubsystem.pivotDownCommand()));
        driverController.rightTrigger().onFalse(intakeSubsystem.zeroIntake());

        driverController.x().whileTrue(drivetrain.applyRequest(() -> brake));


        operatorController.button(1).onTrue(shooterSubsystem.unJamShooterCommand());
        operatorController.button(1).onFalse(shooterDefault());
        
        operatorController.button(2).onTrue(intakeSubsystem.unJamIntakeCommand());
        operatorController.button(2).onFalse(intakeSubsystem.zeroIntake());

        operatorController.button(3).onTrue(new InstantCommand(() -> intakeSubsystem.pivotMotor.setPosition(0)));
        operatorController.button(4).onTrue(intakeSubsystem.pivotUpCommand());

        operatorController.button(5).onTrue(drivetrain.toggleOverrideCommand());

        operatorController.button(11).onTrue(shooterSubsystem.incrementMultiplier());
        operatorController.button(12).onTrue(shooterSubsystem.decrementMultiplier());

        driverController.leftBumper().onTrue(manualShoot());
        driverController.leftBumper().onFalse(new InstantCommand(() -> shooterSubsystem.setManual(false)).andThen(shooterDefault()));


        // driverController.y().whileTrue(drivetrain.trenchFlyCommand());

        // new Trigger(shooterSubsystem::shouldUpdateShooter).whileTrue(shooterSubsystem.updateShooterCommand());

        new Trigger(drivetrain::getTeleop).onTrue(shooterDefault());

        // driverController.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));



        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        // driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private Command shoot()
    {
        return hopperSubsystem.positiveEffortCommand()
            .andThen(drivetrain.snapToHubCommandEnd(drive, () -> drivetrain.getOverride()))
            .andThen(Commands.parallel(
                shooterSubsystem.shootCommand(), 
                Commands.sequence(new WaitCommand(ShooterConstants.timeTillOscillation), intakeSubsystem.intakeRunCommand(), intakeSubsystem.oscillatePivotCommand())));
    }
    private Command manualShoot(){
        return hopperSubsystem.positiveEffortCommand()
            .andThen(drivetrain.snapToHubCommandEnd(drive, () -> true))
            .andThen(Commands.parallel(
                shooterSubsystem.manualShotCommand(), 
                Commands.sequence(new WaitCommand(ShooterConstants.timeTillOscillation), intakeSubsystem.intakeRunCommand(), intakeSubsystem.oscillatePivotCommand())));
    }

    private Command autonShoot()
    {
        return hopperSubsystem.positiveEffortCommand()
            .andThen(Commands.parallel(
                shooterSubsystem.shootCommand(), 
                Commands.sequence(new WaitCommand(ShooterConstants.timeTillOscillation), intakeSubsystem.intakeRunCommand(), intakeSubsystem.oscillatePivotCommand())));
    }

    public static Command shooterDefault()
    {
        return hopperSubsystem.zeroEffortCommand()
            .andThen(intakeSubsystem.zeroIntake())
            .alongWith(shooterSubsystem.shooterIdleCommand());
    }

    public Command getAutonomousCommand() {
        return autonChooser.getSelected();
    }
}

