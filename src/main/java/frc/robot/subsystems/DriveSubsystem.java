/* (C) 2021 */
package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.ADIS16448_IMUSim;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {
	// The motors on the left side of the drive.
	VictorSP leftFront = new VictorSP(Constants.DriveConstants.kLeftMotor1Port);
	VictorSP rightFront = new VictorSP(Constants.DriveConstants.kRightMotor1Port);

	// The robot's drive
	private final DifferentialDrive m_drive = new DifferentialDrive(leftFront, rightFront);

	// The left-side drive encoder

	private final Encoder m_leftEncoder = new Encoder(Constants.DriveConstants.kLeftEncoderPorts[0],
			Constants.DriveConstants.kLeftEncoderPorts[1], Constants.DriveConstants.kLeftEncoderReversed);

	// The right-side drive encoder
	private final Encoder m_rightEncoder = new Encoder(Constants.DriveConstants.kRightEncoderPorts[0],
			Constants.DriveConstants.kRightEncoderPorts[1], Constants.DriveConstants.kRightEncoderReversed);

	// The gyro sensor
	private final ADIS16448_IMU m_gyro = new ADIS16448_IMU();

	// Odometry class for tracking robot pose
	private final DifferentialDriveOdometry m_odometry;

	// These classes help us simulate our drivetrain
	public DifferentialDrivetrainSim m_drivetrainSimulator;
	private EncoderSim m_leftEncoderSim;
	private EncoderSim m_rightEncoderSim;

	private Servo servo;
	// The Field2d class shows the field in the sim GUI
	private Field2d m_fieldSim;
	private ADIS16448_IMUSim m_gyroSim;

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		// Sets the distance per pulse for the encoders
		m_leftEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);
		m_rightEncoder.setDistancePerPulse(Constants.DriveConstants.kEncoderDistancePerPulse);

		resetEncoders();
		m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

		if (RobotBase.isSimulation()) { // If our robot is simulated
			// This class simulates our drivetrain's motion around the field.
			m_drivetrainSimulator = new DifferentialDrivetrainSim(Constants.DriveConstants.kDrivetrainPlant,
					Constants.DriveConstants.kDriveGearbox, Constants.DriveConstants.kDriveGearing,
					Constants.DriveConstants.kTrackwidthMeters, Constants.DriveConstants.kWheelDiameterMeters / 2.0,
					VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

			// The encoder and gyro angle sims let us set simulated sensor readings
			m_leftEncoderSim = new EncoderSim(m_leftEncoder);
			m_rightEncoderSim = new EncoderSim(m_rightEncoder);

			/*
			 * leftFront.setInverted(InvertType.None); leftFront.setSensorPhase(false);
			 * leftRear.setInverted(InvertType.None); rightFront.setSensorPhase(false);
			 */
			m_gyroSim = new ADIS16448_IMUSim(m_gyro);

			// the Field2d class lets us visualize our robot in the simulation GUI.
			m_fieldSim = new Field2d();
			SmartDashboard.putData("Field", m_fieldSim);
		}
	}

	@Override
	public void periodic() {
		// Update the odometry in the periodic block}
		m_odometry.update(Rotation2d.fromDegrees(getHeading()), m_leftEncoder.getDistance(),
				m_rightEncoder.getDistance());
		m_fieldSim.setRobotPose(getPose());
	}

	@Override
	public void simulationPeriodic() {
		// To update our simulation, we set motor voltage inputs, update the simulation,
		// and write the simulated positions and velocities to our simulated encoder and
		// gyro.
		// We negate the right side so that positive voltages make the right side
		// move forward.

		m_drivetrainSimulator.setInputs(leftFront.get() * RobotController.getBatteryVoltage(),
				-rightFront.get() * RobotController.getBatteryVoltage());
		m_drivetrainSimulator.update(0.020);

		/*
		 * leftSim.setQuadratureRawPosition(distanceToNativeUnits(m_drivetrainSimulator.
		 * getLeftPositionMeters()));
		 * leftSim.setQuadratureVelocity(velocityToNativeUnits(m_drivetrainSimulator.
		 * getLeftVelocityMetersPerSecond()));
		 * rightSim.setQuadratureRawPosition(distanceToNativeUnits(m_drivetrainSimulator
		 * .getRightPositionMeters()));
		 * rightSim.setQuadratureRawPosition(velocityToNativeUnits(m_drivetrainSimulator
		 * .getRightVelocityMetersPerSecond()));
		 */
		m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
		m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
		m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
		m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());

		/*
		 * leftSim.setBusVoltage(RobotController.getBatteryVoltage());
		 * rightSim.setBusVoltage(RobotController.getBatteryVoltage());
		 */

		m_gyroSim.setGyroAngleZ(-m_drivetrainSimulator.getHeading().getDegrees());
	}

	/**
	 * Returns the current being drawn by the drivetrain. This works in SIMULATION
	 * ONLY! If you want it to work elsewhere, use the code in
	 * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
	 *
	 * @return The drawn current in Amps.
	 */
	public double getDrawnCurrentAmps() {
		return m_drivetrainSimulator.getCurrentDrawAmps();
	}

	public void setFieldTrajectory(Trajectory traj) {
		m_fieldSim.getObject("trayectoria").setTrajectory(traj);
	}

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}

	/**
	 * Generate a Command with an input trajectory and check if it needs an initial
	 * pose
	 *
	 * @param trajectory
	 * @param initPose
	 * @return
	 */
	public Command createCommandForTrajectory(Trajectory trajectory, Boolean initPose) {
		if (initPose) {
			resetOdometry(trajectory.getInitialPose());
		}

		RamseteCommand rCommand = new RamseteCommand(trajectory, this::getPose,
				new RamseteController(Constants.AutoConstants.kRamseteB, Constants.AutoConstants.kRamseteZeta),
				new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts,
						Constants.DriveConstants.kvVoltSecondsPerMeter,
						Constants.DriveConstants.kaVoltSecondsSquaredPerMeter),
				Constants.DriveConstants.kDriveKinematics, this::getWheelSpeeds,
				new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),

				new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0),
				// RamseteCommand passes volts to the callback
				this::tankDriveVolts, this);

		SmartDashboard.putNumber("Tiempo en realizar Trayectoria", trajectory.getTotalTimeSeconds());
		return rCommand.andThen(() -> this.tankDriveVolts(0, 0));
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose
	 *             The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		// resetEncoders();
		m_drivetrainSimulator.setPose(pose);
		m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
	}

	/**
	 * Drives the robot using arcade controls.
	 *
	 * @param fwd
	 *            the commanded forward movement
	 * @param rot
	 *            the commanded rotation
	 */
	public void arcadeDrive(double fwd, double rot) {
		m_drive.arcadeDrive(fwd, rot);
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts
	 *                   the commanded left output
	 * @param rightVolts
	 *                   the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		var batteryVoltage = RobotController.getBatteryVoltage();
		if (Math.max(Math.abs(leftVolts), Math.abs(rightVolts)) > batteryVoltage) {
			leftVolts *= batteryVoltage / 12.0;
			rightVolts *= batteryVoltage / 12.0;
		}
		leftFront.setVoltage(leftVolts);
		rightFront.setVoltage(-rightVolts);
		m_drive.feed();
	}

	/** Resets the drive encoders to currently read a position of 0. */
	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	/**
	 * Gets the average distance of the two encoders.
	 *
	 * @return the average of the two encoder readings
	 */
	public double getAverageEncoderDistance() {
		return (m_leftEncoder.getDistance() + m_rightEncoder.getDistance()) / 2.0;
	}

	/**
	 * Gets the left drive encoder.
	 *
	 * @return the left drive encoder
	 */
	public Encoder getLeftEncoder() {
		return m_leftEncoder;
	}

	/**
	 * Gets the right drive encoder.
	 *
	 * @return the right drive encoder
	 */
	public Encoder getRightEncoder() {
		return m_rightEncoder;
	}

	/**
	 * Sets the max output of the drive. Useful for scaling the drive to drive more
	 * slowly.
	 *
	 * @param maxOutput
	 *                  the maximum output to which the drive will be constrained
	 */
	public void setMaxOutput(double maxOutput) {
		m_drive.setMaxOutput(maxOutput);
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
	}

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from -180 to 180
	 */
	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360) * (Constants.DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	private int distanceToNativeUnits(double positionMeters) {
		double wheelRotations = positionMeters / (2 * Math.PI * Constants.DriveConstants.kWheelDiameterMeters);
		double motorRotations = wheelRotations * Constants.DriveConstants.kDriveGearing;
		int sensorCounts = (int) (motorRotations * Constants.DriveConstants.kEncoderCPR);
		return sensorCounts;
	}

	private double nativeUnitsToDistanceMeters(double sensorCounts) {
		double motorRotations = (double) sensorCounts / Constants.DriveConstants.kEncoderCPR;
		double wheelRotations = motorRotations / Constants.DriveConstants.kDriveGearing;
		double positionMeters = wheelRotations * (2 * Math.PI * Constants.DriveConstants.kWheelDiameterMeters);
		return positionMeters;
	}

	private int velocityToNativeUnits(double velocityMetersPerSecond) {
		double wheelRotationsPerSecond = velocityMetersPerSecond
				/ (2 * Math.PI * Constants.DriveConstants.kWheelDiameterMeters);
		double motorRotationsPerSecond = wheelRotationsPerSecond * Constants.DriveConstants.kDriveGearing;
		double motorRotationsPer100ms = motorRotationsPerSecond / Constants.DriveConstants.k100msPerSecond;
		int sensorCountsPer100ms = (int) (motorRotationsPer100ms * Constants.DriveConstants.kEncoderCPR);
		return sensorCountsPer100ms;
	}
}
