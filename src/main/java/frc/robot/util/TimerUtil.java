package frc.robot.util;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TimerUtil {
    public static boolean wonAuton = false;
    public static boolean hubActive = true;

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


    public static void updateAutonWinner()
    {
        String gameData = DriverStation.getGameSpecificMessage();
        Alliance allianceColor = getAlliance();

        if (gameData.length() > 0)
        {
            switch (gameData.charAt(0)) {
                case 'B':
                    if (allianceColor.equals(Alliance.Blue))
                    {
                        wonAuton = true;
                    } else
                    {
                        wonAuton = false;
                    }
                    break;
            
                case 'R':
                    if (allianceColor.equals(Alliance.Red))
                    {
                        wonAuton = true;
                    } else
                    {
                        wonAuton = false;
                    }
                    break;
            
                default:
                    wonAuton = false;
                    break;
            }
        }
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

