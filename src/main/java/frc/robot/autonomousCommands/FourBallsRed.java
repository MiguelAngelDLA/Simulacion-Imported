/* (C) 2021 */
package frc.robot.autonomousCommands;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FourBallsRed extends SequentialCommandGroup {
	/** Creates a new GalacticSearch. */
	public FourBallsRed(DriveSubsystem m_robotDrive) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		TrajectoryConfig config = new TrajectoryConfig(4.35, 11.82);
		config.setEndVelocity(11.82);
		Trajectory examplePath = PathPlanner.loadPath("4-5 pelotas red", 4.35, 11.82);

		addCommands(m_robotDrive.createCommandForTrajectory(examplePath, true));
	}

}
