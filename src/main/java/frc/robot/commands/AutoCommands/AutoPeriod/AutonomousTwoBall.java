// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.AutoPeriod;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.AutoDrive;
import frc.robot.commands.AutoCommands.TimeDelay;
import frc.robot.commands.ManualMovements.MoveIndexing;
import frc.robot.commands.ManualMovements.RunIntake;
import frc.robot.commands.ManualMovements.Turret.LaunchBall;
import frc.robot.commands.ManualMovements.Turret.LoadShooter;
import frc.robot.commands.Toggles.ToggleIntake;
import frc.robot.subsystems.AutonomousPathDrivetrain;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Indexing;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.IntakeMove;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Turret;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

//Whole class may be unnecessary, and a timed class might be better and just run the method and give it a time you know it will complete in
public class AutonomousTwoBall extends SequentialCommandGroup {
  /** Creates a new PathTwo. */
  public AutonomousTwoBall(DriveTrain driveTrain, Indexing indexing, IntakeMove intakeMove, Intake intake, Shooter shooter, Turret turret, AutonomousPathDrivetrain autoDrive, Limelight limelight) { // might not need to pass subsystems
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ParallelRaceGroup(
                    new ToggleIntake(intakeMove, true, true), 
                    new TimeDelay(1)),
                new ParallelRaceGroup(
                    new RunIntake(intake, true), 
                    new MoveIndexing(indexing), 
                    new AutoDrive(driveTrain, 1.5)),
                new ParallelRaceGroup(
                    new RunIntake(intake, true), 
                    new TimeDelay(2)),
                new AutonomousTurning(autoDrive, driveTrain, 170),
                //new AutoToggleTracking(turret), // if errors uncomment this
                new TimeDelay(0.5),
                new AutoDrive(driveTrain, 0.5),
                new ParallelRaceGroup
                    (new LaunchBall(shooter, limelight, 3400), 
                    new TimeDelay(1)),                     
                new ParallelRaceGroup
                    (new MoveIndexing(indexing), 
                    new LoadShooter(shooter, true), 
                    new TimeDelay(0.175)),
                new TimeDelay(0.5),
                new ParallelRaceGroup
                    (new MoveIndexing(indexing), 
                    new LoadShooter(shooter, true), 
                    new TimeDelay(0.25)));
                    //new AutoToggleTracking(turret));// Deploy intake, run shooter launch motor, drive 50 in, stop, turn 180 degrees, aim, shoot
    // Might need autonomous command versions of everything
  }
}
