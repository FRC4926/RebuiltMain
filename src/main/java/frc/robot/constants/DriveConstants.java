package frc.robot.constants;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;

public class DriveConstants {
    public static final double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    public static final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    public static final PIDController snapToHubPID = new PIDController(8, 0, 0); //5
    public static final double snapToHubRotationTolerance = 1*Math.PI/180.0; //1


}
