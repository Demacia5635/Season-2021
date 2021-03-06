/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.commands.Drive;
import frc.robot.commands.Drive.DriveStates;
import frc.robot.commands.Drive.InputHandler;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Roulette;
import frc.robot.subsystems.Shooting;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot
 * (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveStates dStates = DriveStates.curvatureDrive;
  private final Chassis chassis = new Chassis(dStates);
  private final Drive driveCommand = new Drive(chassis, InputHandler.singer, dStates);
  private final Climb climb = new Climb();
  private final Pickup pickup = new Pickup();
  private final Roulette roulette = new Roulette();
  private final Shooting shooting = new Shooting();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    SmartDashboard.putData(chassis);
    SmartDashboard.putData(climb);
    SmartDashboard.putData(pickup);
    SmartDashboard.putData(roulette);
    SmartDashboard.putData(shooting);
    // Configure the button bindings
    configureButtonBindings();
  }

  public static PigeonIMU gyro;

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  }

  /**
   * Attempts the AutoNav challenge.
   * Opens the json file of the selected path, and follows it.
   * 
   * @return the AutoNav command
   */
  private Command getAutoNavCommand() {
    final String trajectoryJSON = "paths/output/Test.wpilib.json";
    Trajectory trajectory = new Trajectory();

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
    }

    RamseteCommand cmd = new RamseteCommand(trajectory, chassis::getPose,
        new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
        Constants.DRIVE_KINEMATICS, chassis::setVelocity, chassis);

    chassis.resetOdometry(trajectory.getInitialPose());

    return cmd.andThen(() -> chassis.setVelocity(0, 0));
  }

  /**
   * Attempts the Galactic Search challenge.
   * Detects whether the path is path A or B and whether it is the red or blue path.
   * Afterwards it finds the closest ball, drives to it, and picks it up.
   * 
   * @return the Galactic Search command
   */
  private Command getGalacticSearchCommand() {
    return null;
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command[] getAutonomousCommands() {
    return new Command[] { getAutoNavCommand(), getGalacticSearchCommand() };
  }

  public Command[] getTeleopCommands() {
    return new Command[] { driveCommand };
  }
}
