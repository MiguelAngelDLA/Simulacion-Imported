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
public class Path {
	private static Trajectory mTrajectory;

	public static Trajectory generateTrajectory() {
		var trajectoryWaypoints = new ArrayList<Pose2d>();

		var start = new Pose2d(1, 2, new Rotation2d(0));
		trajectoryWaypoints.add(start);
		// var interior1 = new Pose2d(2, 6, new Rotation2d(70));
		// trajectoryWaypoints.add(interior1);
		// var interior2 = new Pose2d(3, 7, new Rotation2d(0));
		// trajectoryWaypoints.add(interior2);
		var enterFirstTurn = new Pose2d(3.6, 2.2, new Rotation2d(0));
		trajectoryWaypoints.add(enterFirstTurn);

		var firstQarterFirstTurn = new Pose2d(4.4, 2, new Rotation2d(Math.toRadians(-45)));
		trajectoryWaypoints.add(firstQarterFirstTurn);

		var secondQuarterFirstTurn = new Pose2d(3.8, 0.8, new Rotation2d(Math.toRadians(-180)));
		trajectoryWaypoints.add(secondQuarterFirstTurn);

		var thirdQuarterFirstTurn = new Pose2d(3.1, 1.6, new Rotation2d(Math.toRadians(-280)));
		trajectoryWaypoints.add(thirdQuarterFirstTurn);

		var firstQuarterSecondTurn = new Pose2d(6, 2.4, new Rotation2d(Math.toRadians(20)));
		trajectoryWaypoints.add(firstQuarterSecondTurn);

		var secondQuarterSecondTurn = new Pose2d(6.8, 3.3, new Rotation2d(Math.toRadians(90)));
		trajectoryWaypoints.add(secondQuarterSecondTurn);

		var thirdQuarterSecondTurn = new Pose2d(6.0, 3.8, new Rotation2d(Math.toRadians(180)));
		trajectoryWaypoints.add(thirdQuarterSecondTurn);

		var fourthQuarterSecondTurn = new Pose2d(5.4, 3, new Rotation2d(Math.toRadians(270)));
		trajectoryWaypoints.add(fourthQuarterSecondTurn);

		var firstQuarterThirdTurn = new Pose2d(7.5, 1, new Rotation2d(Math.toRadians(15)));
		trajectoryWaypoints.add(firstQuarterThirdTurn);

		var secondQuarterThirdTurn = new Pose2d(8.3, 1.6, new Rotation2d(Math.toRadians(90)));
		trajectoryWaypoints.add(secondQuarterThirdTurn);

		var thirdQuarterThirdTurn = new Pose2d(7.5, 2.4, new Rotation2d(Math.toRadians(180)));
		trajectoryWaypoints.add(thirdQuarterThirdTurn);

		var end = new Pose2d(1, 2.2, new Rotation2d(Math.toRadians(180)));
		trajectoryWaypoints.add(end);

		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.DriveConstants.kDriveKinematics);
		config.setReversed(false);

		var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, config);

		mTrajectory = trajectory;
		return mTrajectory;
	}

	public Trajectory getTrajectory() {
		if (mTrajectory == null) {
			return generateTrajectory();
		}
		return mTrajectory;
	}
}
