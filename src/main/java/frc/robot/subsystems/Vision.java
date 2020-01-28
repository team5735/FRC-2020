/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.RobotConstants;

public class Vision extends SubsystemBase {
  /**
   * Creates a new Vision.
   * 
   * tv | Whether the limelight has any valid targets (0 or 1) tx | Horizontal
   * Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8
   * to 29.8 degrees) ty | Vertical Offset From Crosshair To Target (LL1: -20.5
   * degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees) ta | Target Area (0%
   * of image to 100% of image) ts | Skew or rotation (-90 degrees to 0 degrees)
   * tl | The pipeline’s latency contribution (ms) Add at least 11ms for image
   * capture latency. tshort | Sidelength of shortest side of the fitted bounding
   * box (pixels) tlong | Sidelength of longest side of the fitted bounding box
   * (pixels) thor | Horizontal sidelength of the rough bounding box (0 - 320
   * pixels) tvert | Vertical sidelength of the rough bounding box (0 - 320
   * pixels) getpipe | True active pipeline index of the camera (0 .. 9) camtran |
   * Results of a 3D position solution, 6 numbers: Translation (x,y,y)
   * Rotation(pitch,yaw,roll)
   */

  // From NetworkTables
  public double tv;
  public double tx;
  public double ty;
  public double ta;
  public double ts;
  public double tl;
  public double tshort;
  public double tlong;
  public double thor;
  public double tvert;
  public double getpipe;
  public double camtran;

  // Calculated
  public double distanceFromCamera;

  public Vision() {

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    double ts = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ts").getDouble(0);
    double tl = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tl").getDouble(0);
    double tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
    double tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
    double thor = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor").getDouble(0);
    double tvert = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert").getDouble(0);
    double getpipe = NetworkTableInstance.getDefault().getTable("limelight").getEntry("getpipe").getDouble(0);
    double camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDouble(0);

    distanceFromCamera = (RobotConstants.TARGETHEIGHTFROMGROUND - RobotConstants.CAMERAHEIGHTFROMGROUND)
        / Math.tan(Math.toRadians(ty) + RobotConstants.CAMERAANGLEFROMPARALLEL);
        
  }
}
