package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;

public class PeriodTimer {

    Timer timer;

    public PeriodTimer(Timer timer) {
        this.timer = timer;
    }

    // @N/A  20s autonomous period
    // @0s   10s transition shift
    // @10s  25s loser shift
    // @35s  25s winner shift
    // @60s  25s loser shift
    // @85s  25s winner shift
    // @110s 30s endgame shift
    // @140s 0s  game end

    public double getPeriodTimer() {
        double time = timer.getMatchTime();
        if(time < 10) {
            return 10-time;
        } else if(time < 35) {
            return 35-time;
        }  else if(time < 60) {
            return 60-time;
        }  else if(time < 85) {
            return 85-time;
        }  else if(time < 110) {
            return 110-time;
        }  else if(time < 140) {
            return 140-time;
        }
        return 0;
    }

    public boolean getController10SecRumble() {
        if (getPeriodTimer() < 10 && getPeriodTimer() > 9.5) {
            return true;
        }
        return false;
    }

    public boolean getController5SecRumble() {
        if (getPeriodTimer() < 5 && getPeriodTimer() > 0) {
            return true;
        }
        return false;
    }
    
}
