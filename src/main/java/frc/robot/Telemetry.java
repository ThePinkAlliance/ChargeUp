// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public class Telemetry {
  public static void logData(String title, Object data, Class<?> parent) {
    Class<?> superClass = parent.getSuperclass();
    String className = parent.getSimpleName();
    String superClassName = superClass != null ? superClass.getSimpleName() : "Unknown";

    log(superClassName, className, title, data);
  }

  private static void log(String prefix, String suffix, String title, Object data) {
    System.out.println("[" + prefix + "::" + suffix + "] " + title + " " + data);
  }
}
