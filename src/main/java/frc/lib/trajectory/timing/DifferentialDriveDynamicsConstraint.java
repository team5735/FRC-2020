package frc.lib.trajectory.timing;

import frc.lib.geometry.ICurvature;
import frc.lib.geometry.IPose2d;
import frc.lib.physics.DifferentialDrive;
import frc.lib.util.Units;
import frc.lib.util.Util;

public class DifferentialDriveDynamicsConstraint<S extends IPose2d<S> & ICurvature<S>> implements TimingConstraint<S> {

    protected final DifferentialDrive drive_;
    protected final double abs_voltage_limit_;

    public DifferentialDriveDynamicsConstraint(final DifferentialDrive drive, double abs_voltage_limit) {
        drive_ = drive;
        abs_voltage_limit_ = abs_voltage_limit;
    }

    @Override
    public double getMaxVelocity(S state) {
        return Units.metersToInches(drive_.getMaxAbsVelocity(
            Units.metersToInches(state.getCurvature()),  // Curvature is in inverse inches, so meters_to_inches is correct.
                /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                abs_voltage_limit_));
    }

    @Override
    public MinMaxAcceleration getMinMaxAcceleration(S state,
                                                    double velocity) {
        // TODO figure out a units convention for generic states.  Traditionally we use inches...
        // NOTE: units cancel on angular velocity.
        DifferentialDrive.MinMax min_max = drive_.getMinMaxAcceleration(new DifferentialDrive.ChassisState(
                        Units.inchesToMeters(velocity), state.getCurvature() * velocity),
                Units.metersToInches(state.getCurvature()),  // Curvature is in inverse inches, so meters_to_inches is correct.
                /*Units.meters_to_inches(Units.meters_to_inches(state.getDCurvatureDs())),  // DCurvature is in inverse inches^2.*/
                abs_voltage_limit_);
        return new MinMaxAcceleration(Units.metersToInches(min_max.min), Units.metersToInches(min_max.max));
    }
}
