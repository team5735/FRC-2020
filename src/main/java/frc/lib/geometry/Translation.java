/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

import frc.lib.util.Util;

/**
 * Add your docs here.
 */
public class Translation implements State {

    protected static final Translation Identity = new Translation();

    protected final double x;
    protected final double y;

    public Translation() {
        this(0.0, 0.0);
    }

    public Translation(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Translation(final Translation other) {
        x = other.x;
        y = other.y;
    }

    public Translation(final Translation start, final Translation end) {
        x = end.x - start.x;
        y = end.y - start.y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public Translation scale(double scalar) {
        return new Translation(x * scalar, y * scalar);
    }

    public Translation translateBy(final Translation other) {
        return new Translation(x + other.x, y + other.y);
    }

    public Translation rotateBy(final Rotation rotation) {
        return new Translation(x * rotation.cosine - y * rotation.sine, x * rotation.sine + y * rotation.cosine);
    }

    public static double dot(final Translation a, final Translation b) {
        return a.x * b.x + a.y * b.y;
    }
    
    public static double cross(final Translation a, final Translation b) {
        return a.x * b.y - a.y * b.x;
    }

    public boolean epsilonEquals(final Translation other, double epsilon) {
        return Util.epsilonEquals(x(), other.x(), epsilon) && Util.epsilonEquals(y(), other.y(), epsilon);
    }

    // @Override
    // public State getIdentity() {
    //     return Identity;
    // }
}
