/* (C) 2021 */
package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomousCommands.FourBallsRed;
import frc.robot.autonomousCommands.ThreeBallsRed;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.trajectories.Path;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the Robot periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands,
 * and button mappings) should be declared here.
 */
public class RobotContainer {
	// The robot's subsystems
	private final DriveSubsystem m_robotDrive = new DriveSubsystem();
	Path trajectory;

	// The driver's controller
	XboxController m_driverController = new XboxController(Constants.OIConstants.kDriverControllerPort);

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	public RobotContainer() {
		// Configure the button bindings
		configureButtonBindings();

		// Configure default commands
		// Set the default drive command to split-stick arcade drive
		m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive, m_driverController));
		// A split-stick arcade command, with forward/backward controlled by the left
		// hand, and turning controlled by the right.

	}

	/**
	 * Use this method to define your button->command mappings. Buttons can be
	 * created by instantiating a {@link GenericHID} or one of its subclasses
	 * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
	 * calling passing it to a {@link JoystickButton}.
	 */
	private void configureButtonBindings() {
		// Drive at half speed when the right bumper is held
		new JoystickButton(m_driverController, 1).whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
				.whenReleased(() -> m_robotDrive.setMaxOutput(1));
	}

	public DriveSubsystem getRobotDrive() {
		return m_robotDrive;
	}

	/** Zeros the outputs of all subsystems. */
	public void zeroAllOutputs() {
		m_robotDrive.tankDriveVolts(0, 0);
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		// Run path following command, then stop at the end.
		return new ThreeBallsRed(m_robotDrive);
	}
}
