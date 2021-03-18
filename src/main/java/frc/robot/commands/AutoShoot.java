// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.HashMap;
import java.util.Scanner;

import java.io.File;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurnToPos.TurnType;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Shooting;

public class AutoShoot extends CommandBase {

  private Shooting shooting;
  private Chassis chassis;
  private SequentialCommandGroup command;

  private boolean firstTime;
  private int cycle;

  private HashMap<Integer, double[]> dictionary;

  public AutoShoot(Shooting shooting, Chassis chassis) {
    this.shooting = shooting;
    this.chassis = chassis;
    dictionary = new HashMap<Integer, double[]>();
    readFile();
  }

  @Override
  public void initialize() {
    command = new TurnToPos(chassis, TurnType.Passive, this::getVisionAngle)
        .andThen(new Shoot(shooting, this::getVel, this::getAngle)
            .alongWith(new TurnToPos(chassis, TurnType.Active, this::getVisionAngle)));
    command.schedule();
  }

  @Override
  public void execute() {
  }

  @Override
  public void end(boolean interrupted) {
    command.cancel();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  /**
   * 
   * @return The angle to the basket/hole.
   */
  private double getVisionAngle() {
    return SmartDashboard.getNumber("Angle", 0);
  }

  /**
   * Calculates the estimated velocity depending on the distance.
   * 
   * @return The velocity of the big wheel.
   */
  private double getVel() {
    int distance = (int) SmartDashboard.getNumber("Distance", 0);
    int distance1 = distance - distance % 50;
    int distance2 = distance1 + 50;
    double vel1 = dictionary.get(distance1)[0];
    double vel2 = dictionary.get(distance2)[0];
    return vel1 + ((distance - distance1) / 50.) * (vel2 - vel1);
  }

  /**
   * Calculates the estimated angle depending on the distance.
   * 
   * @return The angle of the hood.
   */
  private double getAngle() {
    int distance = (int) SmartDashboard.getNumber("Distance", 0);
    int distance1 = distance - distance % 50;
    int distance2 = distance1 + 50;
    double angle1 = dictionary.get(distance1)[1];
    double angle2 = dictionary.get(distance2)[1];
    return angle1 + ((distance - distance1) / 50.) * (angle2 - angle1);
  }

  /**
   * Reads the data written in the json file located in Utils/Shooting.json and
   * transferrs it to an HashMap.
   */
  private void readFile() {
    firstTime = true;
    cycle = 0;
    int distance = -1;
    double velocity = -1, angle = -1;
    try {
      Scanner scanner = new Scanner(new File(".\\utils\\Shooting.json"));
      while (scanner.hasNextLine()) {
        String str = scanner.nextLine();
        String value;
        if (str.contains(":") && !firstTime) {
          value = str.split(":", 2)[1].split("\"", 3)[1];
          if (cycle == 0) {
            distance = Integer.parseInt(value);
            cycle++;
          } else if (cycle == 1) {
            velocity = Double.parseDouble(value);
            cycle++;
          } else {
            angle = Double.parseDouble(value);
            cycle = 0;
            dictionary.put(distance, new double[] { velocity, angle });
          }
        } else if (firstTime && str.contains(":")) firstTime = false;
      }
      scanner.close();
    } catch (Exception e) {
      System.out.println("Couldn't access file");
    }
  }
}
