package frc.robot.constants;

import com.pathplanner.lib.config.PIDConstants;

public class AutonConstants {
        public static final PIDConstants pathplannerTranslationPIDConstants = new PIDConstants(3, 0, 0);
        public static final PIDConstants pathplannerRotationPIDConstants    = new PIDConstants(5, 0, 0);
}
