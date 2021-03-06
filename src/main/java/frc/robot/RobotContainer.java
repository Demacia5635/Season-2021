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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
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
  private XboxController mainController = new XboxController(Constants.XBOX_PORT);
  private JoystickButton visionPickupButton;
  private final Chassis chassis = new Chassis();
  private final Drive driveCommand = new Drive(chassis, mainController);
  private final Climb climb = new Climb();
  private final Pickup pickup = new Pickup();
  private final Roulette roulette = new Roulette();
  private final Shooting shooting = new Shooting();
  private JoystickButton shootButton;
  private JoystickButton collectButton;
  private JoystickButton revRouletteButton;
  AutoShoot shoot;
  public static PigeonIMU gyro;
  private Command autCommand;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    shoot = new AutoShoot(shooting, chassis);
    SmartDashboard.putData(chassis);
    SmartDashboard.putData(climb);
    SmartDashboard.putData(pickup);
    SmartDashboard.putData(roulette);
    SmartDashboard.putData(shooting);
    // Configure the button bindings
    configureButtonBindings();

    // autCommand = new Calibrate(this.chassis);
    autCommand = getAutoNavCommand();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    visionPickupButton = new JoystickButton(mainController, XboxController.Button.kA.value);
    visionPickupButton
        .whenHeld(pickup.getPickupCommand().alongWith(chassis.driveToBallCommand(0.5)));
    revRouletteButton = new JoystickButton(mainController, XboxController.Button.kBack.value);
    revRouletteButton.whenPressed(new InstantCommand(() -> roulette.setRouletteVel(30)));
    revRouletteButton.whenReleased(new InstantCommand(() -> roulette.setRouletteVel(-30)));
    collectButton = new JoystickButton(mainController, XboxController.Button.kX.value);
    collectButton.whenHeld(pickup.getPickupCommand());
    shootButton = new JoystickButton(mainController, XboxController.Button.kB.value);
    shootButton.whenHeld(shoot);

  }

  /**
   * Attempts the AutoNav challenge.
   * Opens the json file of the selected path, and follows it.
   * 
   * @return the AutoNav command
   */
  private Command getBounceCommand() {
    final String[] bouncers = new String[] { "paths/output/bouncer1.wpilib.json",
                                             "paths/output/bouncer2.wpilib.json",
                                             "paths/output/bouncer3.wpilib.json",
                                             "paths/output/bouncer4.wpilib.json" };

    Trajectory[] trajectory = new Trajectory[4];

    for (int i = 0; i < bouncers.length; i++) {
      try {
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(bouncers[i]);
        trajectory[i] = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException ex) {
        DriverStation.reportError("Unable to open trajectory: " + bouncers[i], ex.getStackTrace());
      }
    }

    Command cmd = null;
    for (Trajectory t : trajectory) {
      if (cmd == null) {
        cmd = new OurRamseteCommand(t, chassis::getPose,
            new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
            Constants.DRIVE_KINEMATICS, chassis::setVelocityOurFF, chassis);
      } else {
        cmd = cmd.andThen(chassis.getReverseCommand())
            .andThen(new OurRamseteCommand(t, chassis::getPose,
                new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
                Constants.DRIVE_KINEMATICS, chassis::setVelocityOurFF, chassis));
      }
    }
    return cmd.andThen(() -> chassis.setVelocity(0, 0));
  }

  private Command getAutoNavCommand() {
    final String path = "paths/output/Slalom.wpilib.json";

    Trajectory trajectory;

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      return new OurRamseteCommand(trajectory, chassis::getPose,
          new RamseteController(Constants.RAMSETE_B, Constants.RAMSETE_ZETA),
          Constants.DRIVE_KINEMATICS, chassis::setVelocityOurFF, chassis)
              .andThen(() -> chassis.setVelocity(0, 0));
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
    }

    return null;
  }

  /**
   * Attempts the Galactic Search challenge.
   * Detects whether the path is path A or B and whether it is the red or blue
   * path.
   * Afterwards it finds each power cell, drives to it, and picks it up.
   * 
   * @return the Galactic Search command
   */
  private Command getGalacticSearchCommand() {
    return SequentialCommandGroup.sequence(/* new ArmChange(ArmChange.Position.Bottom, pickup), */
        pickup.getPickupCommand(), new SelectCommand(() -> {
          double angleToNearestBall = SmartDashboard.getNumber("BallAngle", 0);
          double distanceToNearestBall = SmartDashboard.getNumber("BallDistance", 0);

          // Path A
          if (Math.abs(angleToNearestBall) <= 2) {
            // Red path
            if (distanceToNearestBall < Constants.CHALLENGE_SPACE_WIDTH / 2) {
              return SequentialCommandGroup.sequence(
                  chassis.driveToBallCommand(Constants.MAX_AUTOMATION_VELOCITY),
                  chassis.findAndDriveToBall(true), chassis.findAndDriveToBall(false));
            }

            // Blue path
            return SequentialCommandGroup.sequence(
                chassis.driveToBallCommand(Constants.MAX_AUTOMATION_VELOCITY),
                chassis.findAndDriveToBall(false), chassis.findAndDriveToBall(false));
          }

          // Path B

          // Red path
          if (distanceToNearestBall * Math.cos(Math.toRadians(angleToNearestBall)) <= 140
              * Constants.INCHES_TO_METERS) {
            return SequentialCommandGroup.sequence(chassis.findAndDriveToBall(true),
                chassis.findAndDriveToBall(false), chassis.findAndDriveToBall(false));
          }

          // Blue path
          return SequentialCommandGroup.sequence(chassis.findAndDriveToBall(true),
              chassis.findAndDriveToBall(false), chassis.findAndDriveToBall(true));
        }) /* , new ArmChange(ArmChange.Position.Top, pickup) */);
  }

  public static int getGalacticPath() {
    double angleToNearestBall = SmartDashboard.getNumber("BallAngle", 0);
    double distanceToNearestBall = SmartDashboard.getNumber("BallDistance", 0);

    if (distanceToNearestBall == 0) {
      return 0;
    }

    // Path A
    if (Math.abs(angleToNearestBall) <= 2) {
      if (distanceToNearestBall < Constants.CHALLENGE_SPACE_WIDTH / 2) {
        return 1;
      }

      // Blue path
      return 2;
    }

    // Path B

    // Red path
    if (distanceToNearestBall * Math.cos(Math.toRadians(angleToNearestBall)) <= 140
        * Constants.INCHES_TO_METERS) {
      return 3;
    }

    // Blue path
    return 4;

  }

  public double getAngle() {
    return SmartDashboard.getNumber("ShootAngle", 5);
  }

  public double getVel() {
    return SmartDashboard.getNumber("ShootVel", 4500);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command[] getAutonomousCommands() {
    return new Command[] {
                           // pickup.getarmMoveCommand()
                           autCommand };
  }

  public Command[] getTeleopCommands() {
    return new Command[] { driveCommand };
  }
}
