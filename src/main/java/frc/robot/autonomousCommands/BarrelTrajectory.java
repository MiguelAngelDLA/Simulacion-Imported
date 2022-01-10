/* (C) 2021 */
package frc.robot.autonomousCommands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class BarrelTrajectory extends CommandBase {

	Trajectory mTrajectory;
	double mTotalTime, mStartTime, mAfterSeconds;
	Timer timer;

	public BarrelTrajectory(Trajectory trajectory, double startTime, double afterSeconds) {
		// Use addRequirements() here to declare subsystem dependencies.
		mTrajectory = trajectory;
		mStartTime = startTime;
		mAfterSeconds = afterSeconds;
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		timer = new Timer();
		timer.start();
		mTotalTime = mTrajectory.getTotalTimeSeconds();
		mStartTime *= mTotalTime;
		mAfterSeconds += mTotalTime;
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		SmartDashboard.putNumber("Timer", timer.get());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return timer.get() > mAfterSeconds;
	}
}
