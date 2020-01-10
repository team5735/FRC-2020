/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.lib.geometry;

/**
 * Add your docs here.
 */
public class Pose {

    public static final Pose Identity = new Pose();

    protected final Translation translation;
    protected final Rotation rotation;

    public Pose() {
        this(new Translation(), new Rotation());
    }

    public Pose(double x, double y, Rotation rotation) {
        this(new Translation(x, y), rotation);

    }

    public Pose(final Pose other) {
        translation = new Translation(other.translation);
        rotation = new Rotation(other.rotation);
    }

    public Pose(Translation translation, Rotation rotation) {
        this.translation = translation;
        this.rotation = rotation;
    }

    public static Pose fromTranslation(final Translation translation) {
        return new Pose(translation, new Rotation());
    }

    public static Pose fromRotation(final Rotation rotation) {
        return new Pose(new Translation(), rotation);
    }

}
