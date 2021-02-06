// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  NetworkTable limeLight;
  NetworkTableEntry tv, tx, ty, ta;
  double d;
  public Limelight() {
    d = 0;
    limeLight = NetworkTableInstance.getDefault().getTable("limelight");
    tv = limeLight.getEntry("tv");
    tx = limeLight.getEntry("tx");
    ty = limeLight.getEntry("ty");
    ta = limeLight.getEntry("ta");
  }

  /*
  if (ta >= minDistance){
    shooter.set(0.9);
  }
  else
  shooter.set(0.5);
  */
  
 // d = (h2-h1)/math.Tan(a1 + a2);



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}
