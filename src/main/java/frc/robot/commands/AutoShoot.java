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

  private final double[] velocities = new double[] { 5500, 3800, 3800, 3900, 4300, 4400, 5000, 5250,
                                                     5500, 6000, 6200, 7300, 7800 };
  private final double[] angles = new double[] {0, 0 , 2, 4, 5, 7, 9, 10, 11, 11, 11, 12, 13};

  public AutoShoot(Shooting shooting, Chassis chassis) {
    this.shooting = shooting;
    this.chassis = chassis;
    //dictionary = new HashMap<Integer, double[]>();
    //readFile();
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
    return shooting.getVisionAngle();
  }

  /**
   * Calculates the estimated velocity depending on the distance.
   * 
   * @return The velocity of the big wheel.
   */
  private double getVel() {
    double distance = (shooting.getVisionDistance() * 100. - 110.);
    int distance1 = (int) ((distance - distance % 50) / 50);
    int distance2 = distance1 + 1;
    double vel1 = velocities[distance1];
    double vel2 = velocities[distance2];
    double vel = vel1 + ((distance % 50 / 50)) * (vel2 - vel1);
    System.out.println(vel);
    return vel;
  }

  /**
   * Calculates the estimated angle depending on the distance.
   * 
   * @return The angle of the hood.
   */
  private double getAngle() {
    double distance = (shooting.getVisionDistance() * 100. - 110.);
    int distance1 = (int) ((distance - distance % 50) / 50);
    int distance2 = distance1 + 1;
    System.out.println(distance1);
    double angle1 = angles[distance1];
    double angle2 = angles[distance2];
    double angle = angle1 + ((distance % 50 / 50)) * (angle2 - angle1);
    System.out.println(angle);
    return angle;
  }

  /**
   * Reads the data written in the json file located in Utils/Shooting.json and
   * transferrs it to an HashMap.
   */
  private void readFile() {
    System.out.println("Got Here");
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
            System.out.println("Distance : " + value);
          } else if (cycle == 1) {
            velocity = Double.parseDouble(value);
            cycle++;
            System.out.println("Velocity : " + value);
          } else {
            angle = Double.parseDouble(value);
            cycle = 0;
            dictionary.put(distance, new double[] { velocity, angle });
            System.out.println("Angle : " + value);
          }
        } else if (firstTime && str.contains(":")) firstTime = false;
      }
      scanner.close();
    } catch (Exception e) {
      System.out.println("Couldn't access file");
    }
  }
}
