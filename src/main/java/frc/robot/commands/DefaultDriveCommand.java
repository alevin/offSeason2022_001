package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DRIVE;
import frc.swervelib.SwerveSubsystem;

public class DefaultDriveCommand extends CommandBase {
    private final SwerveSubsystem m_swerveSubsystem;
    private final XboxController m_controller;
    private static final SendableChooser<String> driverChooser = new SendableChooser<>();
    private static final SendableChooser<String> orientationChooser = new SendableChooser<>();

    private double m_translationX;
    private double m_translationY;
    private double m_rotation;

    public DefaultDriveCommand(SwerveSubsystem swerveSubsystem,
                               XboxController controller) {
        this.m_swerveSubsystem = swerveSubsystem;
        this.m_controller = controller;

        // Control Scheme Chooser
        driverChooser.setDefaultOption("Both Sticks", "Both Sticks");
        driverChooser.addOption("Left Stick and Triggers", "Left Stick and Triggers");
        driverChooser.addOption("Split Sticks and Triggers", "Split Sticks and Triggers");
        driverChooser.addOption("Gas Pedal", "Gas Pedal");
        SmartDashboard.putData("Driver Chooser", driverChooser);

        // Control Orientation Chooser
        orientationChooser.setDefaultOption("Field Oriented", "Field Oriented");
        orientationChooser.addOption("Robot Oriented", "Robot Oriented");
        SmartDashboard.putData("Orientation Chooser", orientationChooser);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        switch (driverChooser.getSelected()) {
            case "Both Sticks":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getLeftX());
              m_rotation = modifyAxis(-m_controller.getRightX());
              break;
            case "Left Stick and Triggers":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getLeftX());
              m_rotation = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
              break;
            case "Split Sticks and Triggers":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getRightX());
              m_rotation = m_controller.getRightTriggerAxis() - m_controller.getLeftTriggerAxis();
              break;
            case "Gas Pedal":
              m_translationX = modifyAxis(-m_controller.getLeftY());
              m_translationY = modifyAxis(-m_controller.getLeftX());
              double angle = calculateTranslationDirection(m_translationX, m_translationY);
              m_translationX = Math.cos(angle) * m_controller.getRightTriggerAxis();
              m_translationY = Math.sin(angle) * m_controller.getRightTriggerAxis();
              m_rotation = modifyAxis(m_controller.getRightX());
              break;
        }

        switch (orientationChooser.getSelected()) {
          case "Field Oriented":
            m_swerveSubsystem.dt.setModuleStates(
                DRIVE.KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            m_translationX * DRIVE.MAX_FWD_REV_SPEED_MPS,
                            m_translationY * DRIVE.MAX_STRAFE_SPEED_MPS,
                            m_rotation * DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC,
                            m_swerveSubsystem.dt.getGyroscopeRotation()
                    )
                )    
            );
            break;
          case "Robot Oriented":
            m_swerveSubsystem.dt.setModuleStates(
              DRIVE.KINEMATICS.toSwerveModuleStates(
                  new ChassisSpeeds(
                          m_translationX * DRIVE.MAX_FWD_REV_SPEED_MPS,
                          m_translationY * DRIVE.MAX_STRAFE_SPEED_MPS,
                          m_rotation * DRIVE.MAX_ROTATE_SPEED_RAD_PER_SEC
                  )
              )    
            );
            break;
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveSubsystem.dt.setModuleStates(
          DRIVE.KINEMATICS.toSwerveModuleStates(  
            new ChassisSpeeds(0.0, 0.0, 0.0)
          )    
        );
    }

  private static double modifyAxis(double value) {
      // Square the axis
      value = Math.copySign(value * value, value);

      return value;
  }

  /**
     * Calculates the angle of translation set by the left stick.
     *
     * @return The angle of translation. 0 corresponds to forwards, and positive
     *     corresponds to counterclockwise.
    */
  private double calculateTranslationDirection(double x, double y) {
    // Calculate the angle.
    // Swapping x/y and inverting y because our coordinate system has +x forwards and -y right
    return Math.atan2(x, -y);
  }
}