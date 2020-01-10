package frc.lib.util;

/**
 * A drivetrain command consisting of the left, right, and normal motor settings
 * inches per second2
 */
public class DriveSignal {
    protected double left;
    protected double right;
    protected double normal; // right is positive

    public DriveSignal(double left, double right) {
        this(left, right, 0);
    }

    public DriveSignal(double left, double right, double normal) {
        this.left = left;
        this.right = right;
        this.normal = normal;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0, 0);

    public double getLeft() {
        return left;
    }

    public double getRight() {
        return right;
    }

    public double getNormal() {
        return normal;
    }

    @Override
    public String toString() {
        return "L: " + left + ", R: " + right + " C:" + normal;
    }
}