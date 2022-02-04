// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomousCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

public class TwoBallsRedv1 extends SequentialCommandGroup {
  /** Creates a new TwoBallsRedv1. */
  public TwoBallsRedv1(DriveSubsystem m_robotDrive) {
    // Use addRequirements() here to declare subsystem dependencies.
    TrajectoryConfig config = new TrajectoryConfig(4.35, 11.82);
		config.setEndVelocity(11.82);
		Trajectory examplePath = PathPlanner.loadPath("3 pelotas red", 4.35, 11.82);

		addCommands(m_robotDrive.createCommandForTrajectory(examplePath, true));
  }

}
