// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

private static final double MAX_SYSTEM_CURRENT_A = 2.0;          // maximum allowed current (A)
private static final double PER_COLOR_CURRENT_A = 0.02;         // 20 mA per color channel at full (A)

private AddressableLED led = new AddressableLED(0);
private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(300-13); 

// Create an LED pattern that will display a rainbow across
  // all hues at maximum saturation and half brightness
  private final LEDPattern m_rainbow = LEDPattern.rainbow(255, 128);

  // Our LED strip has a density of 60 LEDs per meter
  private static final Distance kLedSpacing = Meters.of(1 / 60.0);

  // Create a new pattern that scrolls the rainbow pattern across the LED strip, moving at a speed
  // of 1 meter per second.
  private final LEDPattern scrollingRainbow =
      m_rainbow.scrollAtAbsoluteSpeed(MetersPerSecond.of(1), kLedSpacing);

public LED() {
    led.setLength(ledBuffer.getLength());   

    // default color: full green (will be scaled to respect current limit)
    setColor(0, 255, 0);

    led.setData(ledBuffer);
    led.start();
}


@Override
public void periodic() {
    // Update the buffer with the rainbow animation
   // scrollingRainbow.applyTo(ledBuffer);
    // Set the LEDs
    led.setData(ledBuffer);
}

/**
 * Set the LED strip to an RGB color, but scale the RGB values so total
 * current for all LEDs never exceeds MAX_SYSTEM_CURRENT_A.
 *
 * Each color channel at 255 uses ~20mA per LED. Total per-LED current =
 * 0.02 * (r/255 + g/255 + b/255). Scale down uniformly if needed.
 */
public void setColor(int r, int g, int b) {
    int len = ledBuffer.getLength();
    if (len <= 0) return;

    double rNorm = clamp01(r / 255.0);
    double gNorm = clamp01(g / 255.0);
    double bNorm = clamp01(b / 255.0);

    double sum = rNorm + gNorm + bNorm; // how many full-color channels are used (0..3)
    double scale = 1.0;

    if (sum > 0.0) {
        // total current if unscaled = len * PER_COLOR_CURRENT_A * sum
        // required scale so total <= MAX_SYSTEM_CURRENT_A:
        double requiredScale = MAX_SYSTEM_CURRENT_A / (len * PER_COLOR_CURRENT_A * sum);
        if (requiredScale < 1.0) scale = requiredScale;
    }

    int rScaled = (int) Math.round(r * scale);
    int gScaled = (int) Math.round(g * scale);
    int bScaled = (int) Math.round(b * scale);

    for (int i = 0; i < len; i++) {
        ledBuffer.setRGB(i, rScaled, gScaled, bScaled);
    }
    led.setData(ledBuffer);
}

private double clamp01(double v) {
    if (v < 0.0) return 0.0;
    if (v > 1.0) return 1.0;
    return v;
}
}
