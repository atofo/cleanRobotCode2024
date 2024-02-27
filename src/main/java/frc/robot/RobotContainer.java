// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Commands.DriveWithJoystick;
import frc.robot.Subsystems.DrivetrainSubsystem;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;

public class RobotContainer {

  private final CommandXboxController m_driverController = new CommandXboxController(OperatorConstants.controllerPort);

  private Trigger LB = m_driverController.leftBumper();
  private Trigger RB = m_driverController.rightBumper();

  private Trigger xButton = m_driverController.x();

  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final Command m_spinUpShooter = Commands.runOnce(m_shooter::enable, m_shooter);
  private final Command m_stopShooter = Commands.runOnce(m_shooter::disable, m_shooter);

  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private final DriveWithJoystick m_DriveWithJoystick = new DriveWithJoystick(m_drivetrainSubsystem,
      () -> -m_driverController.getRawAxis(0), 
      () -> m_driverController.getRawAxis(1),
      () -> m_driverController.getRawAxis(4), 
      () -> m_driverController.getRawAxis(3),
      () -> m_driverController.getRawAxis(2));

  

  public RobotContainer() {
    m_drivetrainSubsystem.setDefaultCommand(m_DriveWithJoystick);
    configureBindings();
  }

  private void configureBindings() {
  RB.onTrue(m_spinUpShooter); //Empezar a girar lanzador
  LB.onTrue(m_stopShooter); //Parar lanzador

  Command shoot =
      Commands.either(
            // Run the feeder
            Commands.runOnce(m_IntakeSubsystem::throwNote, m_IntakeSubsystem), //Va a lanzar si se alcanza la velocidad deseada
            // Do nothing
            Commands.none(),
            // Determine which of the above to do based on whether the shooter has reached the
            // desired speed
            m_shooter::atSetpoint);

    Command stopIntake = Commands.runOnce(m_IntakeSubsystem::intakeOFF, m_IntakeSubsystem);

    //disparar cuando se presione el boton X
    xButton.onTrue(shoot).onFalse(stopIntake);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
