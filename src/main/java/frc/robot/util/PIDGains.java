package frc.robot.util;

public class PIDGains {
    
    public final double p;
    public final double i;
    public final double d;
    public final double FF;
    public int numArgs;

    public PIDGains(double p, double i, double d) {
        this(p, i, d, 0);
        this.numArgs = 3;
    }

    public PIDGains(double p, double i, double d, double FF) {
        this.p = p;
        this.i = i;
        this.d = d;
        this.FF = FF;
        this.numArgs = 4;
    }
    
}