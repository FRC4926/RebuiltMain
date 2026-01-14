package frc.robot.subsystems;

import java.util.function.IntSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.AutonConstants;
import frc.robot.util.LimelightHelpers;

public class ObjectDetection extends SubsystemBase {

    private double tX = LimelightHelpers.getTX("");
    private double tY = LimelightHelpers.getTY("");
    private double tA = LimelightHelpers.getTA("");
    boolean hasTarget = LimelightHelpers.getTV("");

    private final PIDController rotationController = AutonConstants.objectDetectionRotationPIDController;

    private Pose2d targetPose = new Pose2d();

    public ObjectDetection() {
        rotationController.setTolerance(AutonConstants.objectDetectionRotationTolerance);
        LimelightHelpers.setPipelineIndex("", 0);
    }

    public RobotCentric rotateToObject(CommandSwerveDrivetrain drivetrain, RobotCentric drive) {
        double measurement = getTX();
        double setpoint = 0;

        double calculation = rotationController.calculate(measurement, setpoint);

        return drive
            .withRotationalRate(calculation);
    }

    public boolean rotFinished()
    {
        return rotationController.atSetpoint();
    }

    public Command autoRotateCommand(CommandSwerveDrivetrain drivetrain, RobotCentric drive)
    {
        return drivetrain.applyRequest(() -> rotateToObject(drivetrain, drive)).until(this::rotFinished);
    }

    public double getTX()
    {
        return tX;
    }


    @Override
    public void periodic() {
        hasTarget = LimelightHelpers.getTV("");

        if (hasTarget)
        {
            tX = LimelightHelpers.getTX("");
            tY = LimelightHelpers.getTY("");
            tA = LimelightHelpers.getTA("");
        } else
        {
            tX = 0;
            tY = 0;
            tA = 0;
        }

        
    }
}
