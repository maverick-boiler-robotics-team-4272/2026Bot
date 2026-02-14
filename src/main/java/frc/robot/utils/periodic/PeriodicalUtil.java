package frc.robot.utils.periodic;

import java.util.*;

public class PeriodicalUtil {
    private static List<Periodical> periodicals = new ArrayList<>();

    public static void registerPeriodic(Periodical periodic) {
        periodicals.add(periodic);
    }

    public static void runPeriodics() {
        for(Periodical p : periodicals) {
            p.periodic();
        }
    }
}
