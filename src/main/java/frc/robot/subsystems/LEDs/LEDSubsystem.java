// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LEDs;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new Led. */
  private final CANdle candle;

  //private DigitalInput ledSwitch = new DigitalInput(0);
//  private static boolean notePresent = false;
public LEDSubsystem() {
  candle = new CANdle(10);
  candle.clearAnimation(0);
  candle.configBrightnessScalar(0.1);
  candle.setLEDs(255,0,0);
  CANdleConfiguration ConfigAll = new CANdleConfiguration();
  candle.configAllSettings(ConfigAll);
  
}
// The color below is for testing, remove it whenever you feel like it

public void randomcolor() {
  candle.clearAnimation(0); // this part causes the led to be set off, removing it's old color
  candle.setLEDs(73,96,123); // I change the blue
  candle.configBrightnessScalar(0.1);
}
//This is a test 
public void setOrange() {
  candle.clearAnimation(0); 
  candle.setLEDs(120,90,90);
  candle.configBrightnessScalar(0.1); //this is the brightness
}
public void setRed() {
  candle.clearAnimation(0);
  candle.setLEDs(255,0,0);
  candle.configBrightnessScalar(0.1);
}
public void setBlue() {
  candle.clearAnimation(0);
  candle.setLEDs(64,224,200);
  candle.configBrightnessScalar(0.1);
}
public void setPurple() {
  candle.clearAnimation(0);
  candle.setLEDs(148,0,211);
  candle.configBrightnessScalar(0.1);
}
public void setGreen() {
  candle.clearAnimation(0);
  candle.setLEDs(0,255,0);
  
  candle.configBrightnessScalar(0.1);
}
public void turnOff() {
  candle.setLEDs(0,0,0);
  candle.clearAnimation(0);
}

public void startFireAnimation() {
  FireAnimation fireAnimation = new FireAnimation(
  0.5,
  0.7,
  700,
  0.8,
  0.7,
  false,
  0
  );

  candle.animate(fireAnimation);
}
public void rainbowAnimation() {
  RainbowAnimation rainbowAnimation = new RainbowAnimation(
    1.0,
    0.7,
    500,
    false,
    0
  );
  candle.animate(rainbowAnimation);
}
public void StrobeAnimation() {
  StrobeAnimation strobeAnimation = new StrobeAnimation(
    0,255,0,
    0,0.5,500,0
  );
  candle.animate(strobeAnimation);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("LED/current", candle.getCurrent());
    SmartDashboard.putNumber("LED/temp", candle.getTemperature());
  }

  
}
