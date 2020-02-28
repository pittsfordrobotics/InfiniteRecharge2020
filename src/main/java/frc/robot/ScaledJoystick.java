package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class ScaledJoystick extends Joystick {
    private static final double kDeadZone = 0.25;

    public ScaledJoystick(int port) {
        super(port);
    }

    public double getScaledX() {
        double x = getX();
        if (Math.abs(x) <= kDeadZone) {
            return 0;
        }

        return x * getScaledThrottle();
    }

    public double getScaledY() {
        double y = getY();
        if (Math.abs(y) <= kDeadZone) {
            return 0;
        }

        return -y * getScaledThrottle();
    }

    public double getScaledTwist() {
        double t = getTwist();
        if (Math.abs(t) <= kDeadZone) {
            return 0;
        }

        return t * getScaledThrottle();
    }

    public double getScaledThrottle() {
        return -getThrottle() / 2 + 0.5;
    }
}