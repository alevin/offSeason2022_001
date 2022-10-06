// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.MoveForward;
import frc.robot.commands.SemiCircle;
import frc.swervelib.SwerveDrivetrainModel;
import frc.swervelib.SwerveSubsystem;

public class RobotContainer {
  private static SwerveDrivetrainModel dt;
  private static SwerveSubsystem m_swerveSubsystem;

  private static final XboxController m_controller = new XboxController(0);

  private static final SendableChooser<Command> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    dt = BearSwerveHelper.createBearSwerve();
    m_swerveSubsystem = BearSwerveHelper.createSwerveSubsystem(dt);
    // Set up the default command for the drivetrain.
    // The controls are defined in the command but use both sticks and the triggers on the driver controller
    m_swerveSubsystem.setDefaultCommand(new DefaultDriveCommand(
            m_swerveSubsystem,
            m_controller
    ));

    // Populate Auto Chooser
    autoChooser.setDefaultOption("Move Forward", new MoveForward(m_swerveSubsystem));
    autoChooser.addOption("Semi-Circle", new SemiCircle(m_swerveSubsystem));
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Back button zeros the gyroscope
    // No requirements because we don't need to interrupt anything
    JoystickButton backButton = new JoystickButton(m_controller, Button.kBack.value);
    backButton.whenPressed(m_swerveSubsystem.dt::zeroGyroscope);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autoChooser.getSelected();
  }
}
