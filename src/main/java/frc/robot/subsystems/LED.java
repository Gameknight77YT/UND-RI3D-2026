// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {

private static final double MAX_SYSTEM_CURRENT_A = 2.0;          // maximum allowed current (A)
private static final double PER_COLOR_CURRENT_A = 0.02;         // 20 mA per color channel at full (A)

private AddressableLED led = new AddressableLED(0);
private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(287); 
    // Breathing effect state
    // breathingThread drives a smooth fade in/out at a specified period
    private volatile Thread breathingThread = null;
    private volatile boolean breathing = false;
    // target color for breathing (0..255)
    private volatile int breathR = 0;
    private volatile int breathG = 0;
    private volatile int breathB = 0;
    // breathing period in seconds (full cycle: fade in + fade out)
    private volatile double breathPeriodSeconds = 4.0;

    private boolean isShooting = false;
    private boolean isIntaking = false;
    private boolean stopBreathing = false;
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


public void setGreen() {
    setColor(0, 255, 0);
}

public void setWhite() {
    setColor(255, 255, 255);
}
public void setYellow() {
    setColor(255, 255, 0);
}

public void setIsShooting(boolean shooting) {
    isShooting = shooting;
}
public void setIsIntaking(boolean intaking) {
    isIntaking = intaking;
}
public void setStopBreathing(boolean stop) {
    stopBreathing = stop;
}

public Command setGreenCommand() {
    return this.runOnce(() -> setGreen());
}

public Command setWhiteCommand() {
    return this.runOnce(() -> setWhite());
}

public Command setBreathingCommand(int r, int g, int b, double periodSeconds) {
    return this.runOnce(() -> startBreathing(r, g, b, periodSeconds));
}

public Command stopBreathingCommand() {
    return this.runOnce(() -> stopBreathing());
}

/**
 * Start a breathing effect at the given RGB color. The LEDs will pulse from off to the
 * specified color and back at the given period (seconds) until stopped.
 *
 * @param r red (0-255)
 * @param g green (0-255)
 * @param b blue (0-255)
 * @param periodSeconds full cycle period in seconds (fade in + fade out)
 */
public synchronized void startBreathing(int r, int g, int b, double periodSeconds) {
    // stop any existing breathing thread first
    stopBreathing();

    breathR = Math.max(0, Math.min(255, r));
    breathG = Math.max(0, Math.min(255, g));
    breathB = Math.max(0, Math.min(255, b));
    breathPeriodSeconds = Math.max(0.1, periodSeconds);

    breathing = true;

    breathingThread = new Thread(() -> {
        final long startTime = System.nanoTime();
        final double period = breathPeriodSeconds;
        final int updateMs = 20; // update interval in ms

        while (breathing && !Thread.currentThread().isInterrupted()) {
            double t = (System.nanoTime() - startTime) / 1e9; // seconds
            // phase in [0,1)
            double phase = (t % period) / period;
            // use cosine to get smooth ease-in/out between 0 and 1
            double factor = 0.5 * (1 - Math.cos(2 * Math.PI * phase));

            int rScaled = (int) Math.round(breathR * factor);
            int gScaled = (int) Math.round(breathG * factor);
            int bScaled = (int) Math.round(breathB * factor);

            setColor(rScaled, gScaled, bScaled);

            try {
                Thread.sleep(updateMs);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
    }, "LED-Breathing");
    breathingThread.setDaemon(true);
    breathingThread.start();
}

/**
 * Stop the breathing effect and leave LEDs at the last target breathing color.
 */
public synchronized void stopBreathing() {
    breathing = false;
    if (breathingThread != null) {
        breathingThread.interrupt();
        try {
            breathingThread.join(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        breathingThread = null;
    }
    // ensure final color is the requested target color
    setColor(breathR, breathG, breathB);
}

/**
 * Stop breathing and set LEDs to a specific final color.
 */
public synchronized void stopBreathing(int r, int g, int b) {
    breathing = false;
    if (breathingThread != null) {
        breathingThread.interrupt();
        try {
            breathingThread.join(200);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        breathingThread = null;
    }
    setColor(r, g, b);
}

public boolean isBreathing() {
    return breathing;
}


@Override
public void periodic() {


    /*if(DriverStation.isDisabled()){
        stopBreathing();
        stopBreathing = false;

        setWhite();
    }

    else*/ if(isShooting && !breathing){
        startBreathing(0, 255, 0, .25);
    }

    else if (!isShooting && stopBreathing){
        stopBreathing();
        stopBreathing = false;

    }
    
    else if(isIntaking && !breathing){
        startBreathing(255, 255, 255, .5);
    }

    else if (!isIntaking && stopBreathing){
        stopBreathing();
        stopBreathing = false;
    }

    else if (!isIntaking && !isShooting){
        setGreen();
    }

    

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
