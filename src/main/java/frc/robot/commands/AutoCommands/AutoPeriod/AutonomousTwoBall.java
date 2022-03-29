// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoPeriod;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Whole class may be unnecessary, and a timed class might be better and just run the method and give it a time you know it will complete in
public class AutonomousTwoBall extends SequentialCommandGroup {
  /** Creates a new PathTwo. */
  public AutonomousTwoBall(DriveTrain driveTrain, Indexing indexing, Intake intake, Shooter shooter, Turret turret) { // might not need to pass subsystems
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(); // Deploy intake, run shooter launch motor, drive 50 in, stop, turn 180 degrees, aim, shoot
    // Might need autonomous command versions of everything
  }
}
