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
public class RedPathA {
	public static Trajectory generateTrajectory() {
		var trajectoryWaypoints = new ArrayList<Pose2d>();

		var start = new Pose2d(0.5, 2.2, new Rotation2d(0));
		trajectoryWaypoints.add(start);

		var firstBall = new Pose2d(2.5, 2.2, new Rotation2d(Math.toRadians(0)));
		trajectoryWaypoints.add(firstBall);

		var secondBall = new Pose2d(3.5, 1.5, new Rotation2d(Math.toRadians(0)));
		trajectoryWaypoints.add(secondBall);

		var thirdBall = new Pose2d(4.4, 4, new Rotation2d(Math.toRadians(20)));
		trajectoryWaypoints.add(thirdBall);

		var end = new Pose2d(9, 4, new Rotation2d(Math.toRadians(0)));
		trajectoryWaypoints.add(end);

		TrajectoryConfig config = new TrajectoryConfig(Constants.AutoConstants.kMaxSpeedMetersPerSecond,
				Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(Constants.DriveConstants.kDriveKinematics);
		config.setReversed(false);

		var trajectory = TrajectoryGenerator.generateTrajectory(trajectoryWaypoints, config);

		return trajectory;
	}
}
