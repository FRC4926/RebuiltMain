package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.controller.PIDController;

public class AutonConstants {
        public static final PIDConstants pathplannerTranslationPIDConstants = new PIDConstants(3, 0, 0);
        public static final PIDConstants pathplannerRotationPIDConstants    = new PIDConstants(5, 0, 0);

        public static final PIDController objectDetectionRotationPIDController = new PIDController(0.07, 0, 0);
        public static final double objectDetectionRotationTolerance = 1.0;
}
