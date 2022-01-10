/* (C) 2021 */
package frc.robot.autonomousCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.trajectories.Bounce;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BounceAuto extends SequentialCommandGroup {
	/** Creates a new BounceAuto. */
	public BounceAuto(DriveSubsystem m_robotDrive) {
		// Add your commands in the addCommands() call, e.g.
		// addCommands(new FooCommand(), new BarCommand());

		var ResetOdometry = new InstantCommand(
				() -> m_robotDrive.resetOdometry(Bounce.generateTrajectoryOne().getInitialPose()));

		addCommands(ResetOdometry, m_robotDrive.createCommandForTrajectory(Bounce.generateTrajectoryOne(), false),
				m_robotDrive.createCommandForTrajectory(Bounce.generateTrajectoryTwo(), false),
				m_robotDrive.createCommandForTrajectory(Bounce.generateTrajectoryThree(), false),
				m_robotDrive.createCommandForTrajectory(Bounce.end(), false));
	}
}
