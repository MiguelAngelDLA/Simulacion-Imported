// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomousCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeBallsRed extends SequentialCommandGroup {
  /** Creates a new ThreeBallsRed. */
  public ThreeBallsRed(DriveSubsystem m_robotDrive) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
		TrajectoryConfig config = new TrajectoryConfig(4.35, 11.82);
		config.setEndVelocity(11.82);
		Trajectory examplePath = PathPlanner.loadPath("3 pelotas red", 4.35, 11.82);

		addCommands(m_robotDrive.createCommandForTrajectory(examplePath, true));
  }
}
