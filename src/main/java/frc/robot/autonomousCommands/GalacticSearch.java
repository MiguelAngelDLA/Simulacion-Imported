/* (C) 2021 */
package frc.robot.autonomousCommands;

import com.pathplanner.lib.PathPlanner;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class GalacticSearch extends SequentialCommandGroup {
	/** Creates a new GalacticSearch. */
	public GalacticSearch(DriveSubsystem m_robotDrive) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());
		Trajectory examplePath = PathPlanner.loadPath("New Path", 3.6, 9.75);

		addCommands(m_robotDrive.createCommandForTrajectory(examplePath, true));
	}
}
