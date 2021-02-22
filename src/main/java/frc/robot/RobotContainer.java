/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.Drive;
import frc.robot.commands.Drive.DriveStates;
import frc.robot.commands.Drive.InputHandler;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.Pickup;
import frc.robot.subsystems.Roulette;
import frc.robot.subsystems.Shooting;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveStates dStates = DriveStates.arcadeDrive; 
  private final Chassis chassis = new Chassis(dStates);
  private final Drive driveCommand = new Drive(chassis,InputHandler.singer,dStates);
  private final Climb climb = new Climb();
  private final Pickup pickup = new Pickup();
  private final Roulette roulette = new Roulette();
  private final Shooting shooting = new Shooting();



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command[] getAutonomousCommands() {
    // An ExampleCommand will run in autonomous
    return null;
  }
  
  public Command[] getTeleopCommands(){
    return null;
  }
}
