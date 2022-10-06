package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.swervelib.SwerveSubsystem;

public class SemiCircle extends SequentialCommandGroup {
/*   SwerveDriveKinematicsConstraint6391 constraint = new SwerveDriveKinematicsConstraint6391(DRIVE.KINEMATICS, DRIVE.MAX_FWD_REV_SPEED_MPS);
  TrajectoryConfig6391 config =
        new TrajectoryConfig6391(
                AUTO.kMaxSpeedMetersPerSecond,
                AUTO.kMaxAccelerationMetersPerSecondSquared)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(DRIVE.KINEMATICS)
            .addConstraint(constraint);
  ArrayList<Rotation2d> headings = new ArrayList<Rotation2d>(
        List.of(
          Rotation2d.fromDegrees(180),
          Rotation2d.fromDegrees(90),
          Rotation2d.fromDegrees(0)));
  PathPlannerTrajectory forward =
        TrajectoryGenerator6391.generateTrajectory(
          List.of(
            // Start at the origin facing the +X direction
            new Pose2d(1, 1, Rotation2d.fromDegrees(45)),
            // Dummy interior waypoint
            new Pose2d(4, 4, Rotation2d.fromDegrees(0)),
            // End 3 meters straight ahead of where we started, facing forward
            new Pose2d(7, 1, Rotation2d.fromDegrees(-45))),
            headings,
            config); */
  
  PathPlannerTrajectory forward = PathPlanner.loadPath("SemiCircle", 2.0, 3.0);
          
  public SemiCircle(SwerveSubsystem m_drive) {
    addCommands(
      m_drive.dt.createCommandForTrajectory(forward, m_drive)
    );
  }
}
