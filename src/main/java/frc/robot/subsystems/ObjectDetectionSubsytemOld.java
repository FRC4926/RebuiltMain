// package frc.robot.subsystems;

// import java.util.function.IntSupplier;

// import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.constants.AutonConstants;
// import frc.robot.util.LimelightHelpers;

// public class ObjectDetectionSubsytem extends SubsystemBase {

//     private double tX = LimelightHelpers.getTX("");
//     private double tY = LimelightHelpers.getTY("");
//     private double tA = LimelightHelpers.getTA("");
//     private boolean hasTarget = LimelightHelpers.getTV("");

//     private final PIDController driveController = AutonConstants.objectDetectionDrivePIDController;
//     private final PIDController rotationController = AutonConstants.objectDetectionRotationPIDController;

//     private Pose2d targetPose = new Pose2d();

//     private Timer flickerTimer = new Timer();

//     public ObjectDetectionSubsytem() {
//         flickerTimer.start();
//         driveController.setTolerance(0);
//         rotationController.setTolerance(AutonConstants.objectDetectionRotationTolerance);
//         LimelightHelpers.setPipelineIndex("", 0);
//     }

//     public RobotCentric autoDriveToObject(CommandSwerveDrivetrain drivetrain, RobotCentric drive) 
//     {
//             // double driveMeasurement = getTY();
//             // double driveSetpoint = -27;

//             double rotMeasurement = getTX();
//             double rotSetpoint = 0;

//             // double driveCalculation = driveController.calculate(driveMeasurement, driveSetpoint);
//             double rotCalculation = rotationController.calculate(rotMeasurement, rotSetpoint);


//             return drive
//                 .withVelocityX(1.75) //-driveCalculation)* Math.max(0.1, (Math.min(1, 0.1 / Math.abs(rotCalculation)))))
//                 .withRotationalRate(rotCalculation);
//     }

//     public RobotCentric driveToObject(CommandSwerveDrivetrain drivetrain, RobotCentric drive)     {
//         if (hasTarget)
//         {
//             // double driveMeasurement = getTY();
//             // double driveSetpoint = -27;

//             double rotMeasurement = getTX();
//             double rotSetpoint = 0;

//             // double driveCalculation = driveController.calculate(driveMeasurement, driveSetpoint);
//             double rotCalculation = rotationController.calculate(rotMeasurement, rotSetpoint);


//             return drive
//                 .withVelocityX(1.75) //-driveCalculation)* Math.max(0.1, (Math.min(1, 0.1 / Math.abs(rotCalculation)))))
//                 .withRotationalRate(rotCalculation);
//         } else
//         {
//             return drive
//                 .withVelocityX(0)
//                 .withVelocityY(0)
//                 .withRotationalRate(0);
//         }
//     }

//     public RobotCentric zeroDrive(RobotCentric drive) {
//        return drive
//             .withRotationalRate(0)
//             .withVelocityX(0)
//             .withVelocityY(0);

//     }

//     public boolean rotFinished()
//     {
//         return rotationController.atSetpoint();
//     }

//     public Command autoTrackCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
//     {
//         return drivetrain.applyRequest(() -> driveToObject(drivetrain, drive));
//     }

//     public Command autoTrackAutonCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
//     {
//         return drivetrain.applyRequest(() -> autoDriveToObject(drivetrain, drive)).until(this::notHasTarget);
//     }

//     public Command autonObjectDetect(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
//     {
//         return autoTrackAutonCommand(drivetrain, drive)
//             .andThen(new InstantCommand(() -> zeroDrive(drive)));
//     }
//     public double getTX()
//     {
//         return tX;
//     }

//     public double getTY()
//     {
//         return tY;
//     }

//     public boolean hasTarget()
//     {
//         return hasTarget;
//     }

//     public boolean notHasTarget()
//     {
//         return !hasTarget || getTY() < -25.5;
//     }

//     @Override
//     public void periodic() {


//         if (LimelightHelpers.getTV(""))
//         {
//             flickerTimer.restart();
//             tX = LimelightHelpers.getTX("");
//             tY = LimelightHelpers.getTY("");
//             tA = LimelightHelpers.getTA("");
//         }

//         hasTarget = flickerTimer.get() <= AutonConstants.timeThres;

//         if (!hasTarget)
//         {   
//             tX = 0;
//             tY = 0;
//             tA = 0;
//         }

//         SmartDashboard.putBoolean("HasTarget", hasTarget);
//     }
// }
