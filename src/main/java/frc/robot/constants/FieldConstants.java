package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {

        public static final AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        // AprilTag related constants
        public static final int aprilTagCount = tagLayout.getTags().size();

        public static final double aprilTagWidth = Units.inchesToMeters(6.5);

        // Field dimensions
        public static final double fieldLength = tagLayout.getFieldLength();
        public static final double fieldWidth = tagLayout.getFieldWidth();

        public static final Rectangle2d fieldRect = new Rectangle2d(new Translation2d(), new Translation2d(fieldLength, fieldWidth));

        public static final Translation2d hubCenterBlue = new Translation2d(4.6256194, 4.0346376);

        public static final Translation2d hubCenterRed = new Translation2d(16.5354-4.6256194, 4.0346376);
}
