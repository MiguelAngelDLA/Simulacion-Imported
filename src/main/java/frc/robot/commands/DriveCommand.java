/* (C) 2021 */
package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
	/** Creates a new DriveCommand. */
	DriveSubsystem m_drive;

	XboxController m_controller;

	public DriveCommand(DriveSubsystem drive, XboxController controller) {
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(drive);
		m_drive = drive;
		m_controller = controller;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		m_drive.zeroHeading();
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		m_drive.arcadeDrive(-m_controller.getRawAxis(4), m_controller.getRawAxis(1) * 0.75);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
