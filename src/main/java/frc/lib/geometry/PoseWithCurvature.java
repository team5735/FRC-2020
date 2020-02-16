package frc.lib.geometry;

import frc.lib.util.Util;

import java.text.DecimalFormat;

public class PoseWithCurvature {
    protected static final PoseWithCurvature kIdentity = new PoseWithCurvature();
    protected final Pose pose_;
    protected final double curvature_;
    protected final double dcurvature_ds_;
    public PoseWithCurvature() {
        pose_ = new Pose();
        curvature_ = 0.0;
        dcurvature_ds_ = 0.0;
    }

    public PoseWithCurvature(final Pose pose, double curvature) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public PoseWithCurvature(final Pose pose, double curvature, double dcurvature_ds) {
        pose_ = pose;
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public PoseWithCurvature(final Translation translation, final Rotation rotation, double curvature) {
        pose_ = new Pose(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = 0.0;
    }

    public PoseWithCurvature(final Translation translation, final Rotation rotation, double curvature, double dcurvature_ds) {
        pose_ = new Pose(translation, rotation);
        curvature_ = curvature;
        dcurvature_ds_ = dcurvature_ds;
    }

    public static final PoseWithCurvature identity() {
        return kIdentity;
    }

    
    public final Pose getPose() {
        return pose_;
    }

    
    public PoseWithCurvature transformBy(Pose transform) {
        return new PoseWithCurvature(getPose().transformBy(transform), getCurvature(), getDCurvatureDs());
    }

    
    public PoseWithCurvature mirror() {
        return new PoseWithCurvature(getPose().mirror().getPose(), -getCurvature(), -getDCurvatureDs());
    }

    
    public double getCurvature() {
        return curvature_;
    }

    
    public double getDCurvatureDs() {
        return dcurvature_ds_;
    }

    
    public final Translation getTranslation() {
        return getPose().getTranslation();
    }

    
    public final Rotation getRotation() {
        return getPose().getRotation();
    }

    
    public PoseWithCurvature interpolate(final PoseWithCurvature other, double x) {
        return new PoseWithCurvature(getPose().interpolate(other.getPose(), x),
                Util.interpolate(getCurvature(), other.getCurvature(), x),
                Util.interpolate(getDCurvatureDs(), other.getDCurvatureDs(), x));
    }

    
    public double distance(final PoseWithCurvature other) {
        return getPose().distance(other.getPose());
    }

    
    public boolean equals(final Object other) {
        if (other == null || !(other instanceof PoseWithCurvature)) return false;
        PoseWithCurvature pwc = (PoseWithCurvature) other;
        return getPose().equals(pwc.getPose()) && Util.epsilonEquals(getCurvature(), pwc.getCurvature()) && Util.epsilonEquals(getDCurvatureDs(), pwc.getDCurvatureDs());
    }

    
    public String toString() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toString() + ", curvature: " + fmt.format(getCurvature()) + ", dcurvature_ds: " + fmt.format(getDCurvatureDs());
    }

    
    public String toCSV() {
        final DecimalFormat fmt = new DecimalFormat("#0.000");
        return getPose().toCSV() + "," + fmt.format(getCurvature()) + "," + fmt.format(getDCurvatureDs());
    }
}
