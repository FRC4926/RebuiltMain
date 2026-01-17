package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

public class AutonConstants {
        public static final PIDConstants pathplannerTranslationPIDConstants = new PIDConstants(3, 0, 0);
        public static final PIDConstants pathplannerRotationPIDConstants    = new PIDConstants(5, 0, 0);

        public static final PIDController objectDetectionDrivePIDController = new PIDController(0.25, 0, 0); //0.25

        public static final PIDController objectDetectionRotationPIDController = new PIDController(0.04, 0, 0);
        public static final double objectDetectionRotationTolerance = 0.1;

        public static final double timeThres = 1;


}
