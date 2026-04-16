package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TimerUtil {
    public static boolean wonAuton = false;
    public static boolean hubActive = true;
    public static String FMSworked = "waiting";

   public static double getMatchTimeLeft() {
        return DriverStation.getMatchTime();
    }

    public static boolean getWonAuton()
    {
        return wonAuton;
    }

    public static Alliance getAlliance()
    {
        return (DriverStation.getAlliance().orElse(Alliance.Red));
    }


    public static boolean updateAutonWinner()
    {
        String gameDataS = DriverStation.getGameSpecificMessage();
        if (gameDataS.length() != 1) {
            wonAuton = false;
            FMSworked = "no idea bruh";
            return false;
        }

        char gameData = gameDataS.charAt(0);

        if (gameData != 'R' && gameData != 'B') {
            wonAuton = false;
            FMSworked = "no idea bruh";
            return false;
        }

        Alliance allianceColorA = getAlliance();
        char allianceColor = 'R';
        if (allianceColorA.equals(Alliance.Blue)) {
            allianceColor = 'B';
        }
        
        FMSworked = "win state recieved!";

        wonAuton = allianceColor == gameData;

        return wonAuton;
    }


    public static boolean wonAuton()
    {
        return wonAuton;
    }

    public static boolean hubActive()
    {
        return hubActive;
    }

    public static double getTimeLeft()
    {
        return 0.0;
    }
}

