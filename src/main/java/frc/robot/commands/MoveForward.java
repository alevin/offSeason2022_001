package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class MoveForward extends SequentialCommandGroup {
  PathPlannerTrajectory forward = PathPlanner.loadPath("MoveForward", 2.0, 3.0);
  public MoveForward(SwerveSubsystem m_drive) {
    addCommands(
      new InstantCommand(() -> m_drive.dt.setKnownPose(forward.getInitialPose())),
      m_drive.dt.createCommandForTrajectory(forward, m_drive)
    );
  }
}
