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
public class Slalom {
	private static Trajectory mTrajectory;

	public static Trajectory generateTrajectory() {
		var trajectoryWaypoints = new ArrayList<Pose2d>();

		var start = new Pose2d(1, 0.5, new Rotation2d(0));
		trajectoryWaypoints.add(start);
		// var interior1 = new Pose2d(2, 6, new Rotation2d(70));
		// trajectoryWaypoints.add(interior1);
		// var interior2 = new Pose2d(3, 7, new Rotation2d(0));
		// trajectoryWaypoints.add(interior2);

		var enterFirstSlalom = new Pose2d(3, 2, new Rotation2d(Math.toRadians(45)));
		trajectoryWaypoints.add(enterFirstSlalom);

		var middleFirstSlalom = new Pose2d(5, 2.4, new Rotation2d(Math.toRadians(0)));
		trajectoryWaypoints.add(middleFirstSlalom);

		var endFirstSlalom = new Pose2d(7.5, 1, new Rotation2d(Math.toRadians(-45)));
		trajectoryWaypoints.add(endFirstSlalom);

		var middleTurn = new Pose2d(8.5, 2, new Rotation2d(Math.toRadians(90)));
		trajectoryWaypoints.add(middleTurn);

		var outerTurn = new Pose2d(7.5, 2.6, new Rotation2d(Math.toRadians(180)));
		trajectoryWaypoints.add(outerTurn);

		var enterSecondSlalom = new Pose2d(6.6, 2, new Rotation2d(Math.toRadians(-110)));
		trajectoryWaypoints.add(enterSecondSlalom);

		var middleSecondSlalom = new Pose2d(5, 1, new Rotation2d(Math.toRadians(-180)));
		trajectoryWaypoints.add(middleSecondSlalom);

		var nearend = new Pose2d(2.8, 1.3, new Rotation2d(Math.toRadians(145)));
		trajectoryWaypoints.add(nearend);

		var end = new Pose2d(1.5, 2.4, new Rotation2d(Math.toRadians(125)));
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
