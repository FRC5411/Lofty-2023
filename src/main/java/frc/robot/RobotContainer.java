package frc.robot;
import frc.lib.Telemetry;
import frc.robot.Constants.ARM.positions;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon;
import frc.robot.subsystems.PinchersofPower;

import java.io.File;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Im leaving these ports as magic constants because there's no case where they are not these values
  public static final CommandXboxController driverController = new CommandXboxController(0);
  public static final CommandGenericHID copilotController = new CommandGenericHID(1);

  // The robot's subsystems and commands are defined here...
  private final Pigeon m_gyro = new Pigeon();
  private final Limelight m_limelight = new Limelight();
  //private final LEDs m_LEDs = new LEDs();
  //private final PinchersofPower m_claw = new PinchersofPower();
  //private final Arm m_arm = new Arm(m_claw, m_LEDs, driverController);
  private final Drivetrain m_swerve = new Drivetrain(m_gyro/*m_arm, m_claw, m_limelight*/);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Telemetry.setValue("general/autonomous/availableRoutines", (String[]) Stream.of(new File("/home/lvuser/deploy/").listFiles()).filter(file -> !file.isDirectory()).map(File::getName).collect(Collectors.toSet()).toArray());

    // Configure the button bindings
    configureButtonBindings();

    //m_LEDs.setDefaultCommand(m_LEDs.idle());
    //m_arm.setDefaultCommand(m_arm.defaultCommand());
    m_swerve.setDefaultCommand(new DriveCommand(m_swerve));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverController.a().onTrue(new InstantCommand(m_swerve::zeroGyro));
    driverController.b().onTrue(new InstantCommand(m_swerve::toggleRobotOrient));

    /*copilotController.button(0).whileTrue(m_arm.moveToPositionCommand(positions.Substation));
    copilotController.button(1).whileTrue(m_arm.moveToPositionCommand(positions.Floor));
    copilotController.button(2).whileTrue(m_arm.moveToPositionCommand(positions.ScoreHigh));
    copilotController.button(3).whileTrue(m_arm.moveToPositionCommand(positions.FloorAlt));
    copilotController.button(4).whileTrue(m_arm.moveToPositionCommand(positions.ScoreMid));
    copilotController.button(5).whileTrue(m_arm.moveToPositionCommand(positions.ScoreLow));
    copilotController.button(6).onTrue(m_claw.outtakeCommand());
    copilotController.button(7).onTrue(m_LEDs.turnYellow().alongWith(new InstantCommand( () -> m_claw.setMode("cone"))));
    copilotController.button(8).onTrue(m_LEDs.turnPurple().alongWith(new InstantCommand( () -> m_claw.setMode("cube"))));*/
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_swerve.getAutonomousCommand();
  }
}
