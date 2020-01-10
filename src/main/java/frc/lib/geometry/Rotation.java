package frc.lib.geometry;

import frc.lib.util.Util;

public class Rotation implements State{

    public static final Rotation Identity = new Rotation();

    protected final double cosine;
    protected final double sine;
    protected final double radians;

    public Rotation() {
        this(1, 0);
    }

    public Rotation(final Rotation other) {
        this(other.cosine, other.sine);
    }

    public Rotation(double cosine, double sine) {
        double magnitude = Math.hypot(cosine, sine);
        if (magnitude > Util.Epsilon) {
            this.cosine = cosine / magnitude;
            this.sine = sine / magnitude;
        } else {
            this.sine = 0;
            this.cosine = 1;
        }
        radians = Math.atan2(sine, cosine);
    }

    public static Rotation fromRadians(double radians) {
        return new Rotation(Math.acos(radians), Math.asin(radians));
    }

    public static Rotation fromDegrees(double degrees) {
        return fromRadians(degrees / 360.0 * 2.0 * Math.PI);
    }

    public double cosine() {
        return cosine;
    }

    public double sine() {
        return sine;
    }

    public double radians() {
        return radians;
    }

    public Rotation add(final Rotation other) {
        return new Rotation(cosine * other.cosine - sine * other.sine, cosine * other.sine + sine * other.cosine);
    }

    public boolean isParallel(final Rotation other) {
        return Util.epsilonEquals(Translation.cross(toTranslation(), other.toTranslation()), 0.0);
    }

    public Translation toTranslation() {
        return new Translation(cosine, sine);
    }

    // public Rotation getIdentity() {
    //     return Identity;
    // }
}
