/* (C) 2021 */
package frc.robot.trajectories;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants;
import java.util.ArrayList;

/** Add your docs here. */
public class Bounce {
	private static Trajectory mTrajectory;

	public static Trajectory generateTrajectoryOne() {
		var trajectoryWaypoints = new ArrayList<Pose2d>();

		var start = new Pose2d(1, 2.2, new Rotation2d(0));
		trajectoryWaypoints.add(start);

		var firstTop = new Pose2d(2.5, 3.4, new Rotation2d(Math.toRadians(90)));
		trajectoryWaypoints.add(firstTop);

		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.DriveConstants.kDriveKinematics);
		config.setReversed(false);

		var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, config);

		mTrajectory = trajectory;
		return mTrajectory;
	}

	public static Trajectory generateTrajectoryTwo() {
		var trajectoryWaypoint = new ArrayList<Pose2d>();

		var middle = new Pose2d(3.5, 1.5, new Rotation2d(Math.toRadians(100)));
		trajectoryWaypoint.add(middle);

		var bottomOne = new Pose2d(4, 0.5, new Rotation2d(Math.toRadians(180)));
		trajectoryWaypoint.add(bottomOne);

		var middleTwo = new Pose2d(4.6, 1.5, new Rotation2d(Math.toRadians(-90)));
		trajectoryWaypoint.add(middleTwo);

		var TopTwo = new Pose2d(4.7, 3.4, new Rotation2d(Math.toRadians(-90)));
		trajectoryWaypoint.add(TopTwo);

		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.DriveConstants.kDriveKinematics);
		config.setReversed(true);

		var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoint, config);

		return trajectory;
	}

	public static Trajectory generateTrajectoryThree() {
		var trajectoryWaypoint = new ArrayList<Pose2d>();

		var MiddleThree = new Pose2d(4.5, 1.4, new Rotation2d(Math.toRadians(-90)));
		trajectoryWaypoint.add(MiddleThree);

		var bottomTwo = new Pose2d(4.8, 0.5, new Rotation2d(Math.toRadians(0)));
		trajectoryWaypoint.add(bottomTwo);

		var bottomThree = new Pose2d(6, 0.5, new Rotation2d(Math.toRadians(0)));
		trajectoryWaypoint.add(bottomThree);

		var topThree = new Pose2d(6.8, 3.4, new Rotation2d(Math.toRadians(90)));
		trajectoryWaypoint.add(topThree);

		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.DriveConstants.kDriveKinematics);
		config.setReversed(false);

		var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoint, config);

		return trajectory;
	}

	public static Trajectory end() {
		var trajectoryWaypoints = new ArrayList<Pose2d>();

		var topThree = new Pose2d(6.8, 2.7, new Rotation2d(Math.toRadians(90)));
		trajectoryWaypoints.add(topThree);

		var end = new Pose2d(7.5, 2.3, new Rotation2d(Math.toRadians(160)));
		trajectoryWaypoints.add(end);

		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.DriveConstants.kDriveKinematics);
		config.setReversed(true);

		var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, config);

		mTrajectory = trajectory;
		return mTrajectory;
	}
}
