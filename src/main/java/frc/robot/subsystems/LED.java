// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Ports;
import frc.robot.Constants.LEDConstants;

public class LED extends SubsystemBase {
  /** Creates a new leds. */
  AddressableLED led; 
  AddressableLEDBuffer ledBuffer; 
  Timer timer; 
  int rainbowFirstPixelHue;


  public LED() {
    led = new AddressableLED(Ports.LEDPorts.LED_PORT); 
    ledBuffer = new AddressableLEDBuffer(0); 
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer); 
    led.start(); 
    

  }

  public void setLedPurple(){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 160, 32, 340);
    }
    led.setData(ledBuffer); 
  }

  public void setLedGreen(){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 0, 225, 0);
    }
    led.setData(ledBuffer); 
  }

  public void setDefault(){
    for(var i = 0; i < ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, 0, 0, 0);
    }
    led.setData(ledBuffer);
    System.out.println("Default Working ");
  }


  public void setRainbow(){ 
    timer.start(); 
    if (timer.get() <= 3){
      for(int i = 0; i < ledBuffer.getLength(); i++){ 
        //WHAT KIND OF VARIABLE IS RAINBOW FIRST PIXEL HUE????????????? 
        final int hue = (rainbowFirstPixelHue + (i * 180/ ledBuffer.getLength())) % 180; 
        ledBuffer.setHSV(i, hue, 255, 128); 
      }
      rainbowFirstPixelHue +=3; 
      rainbowFirstPixelHue %= 180;
      led.setData(ledBuffer);
    }
  else {
    timer.stop(); 
    }
    System.out.println("Rainbow Running ");
  }

  public void setGrupleFlicker(){
    timer.start(); 
    if(timer.get()< 3){
      if (timer.get()%2 == 0 ){
        setLedPurple();
          led.setData(ledBuffer);
      }
      else{ 
        setLedGreen();
          led.setData(ledBuffer);
      } 
    }
    System.out.println("Gruple Running"); 
  }

  public void setRedFlicker(){
    timer.start(); 
    if(timer.get()<3){
      if(timer.get()%2 ==0){ 
        for(var i = 0; i<ledBuffer.getLength(); i++){
          ledBuffer.setRGB(i, 255, 0, 0);
        }
        led.setData(ledBuffer);
      }
      else { 
        for(var j = 0; j< ledBuffer.getLength(); j++){
          ledBuffer.setRGB(j, 0, 0, 0);
        }
        led.setData(ledBuffer);
      }
    }
    else {
      timer.stop(); 
    }
    System.out.println("Red Flicker Running ");
  }

  public void setSolid(int[] solid){ 
    // intake 
    timer.start(); 
    if(timer.get()<= 0){
      if(solid ==  LEDConstants.PINK){
        for(var j = 0; j < ledBuffer.getLength(); j++){
          ledBuffer.setRGB(j, LEDConstants.PINK[0], LEDConstants.PINK[1], LEDConstants.PINK[3]);
        }
        led.setData(ledBuffer); 
      }
      // Shooter
      else if(solid == LEDConstants.BLUE){
        for(var j = 0; j< ledBuffer.getLength(); j++){
          ledBuffer.setRGB(j, LEDConstants.BLUE[0], LEDConstants.BLUE[1], LEDConstants.BLUE[2]);
        }
        led.setData(ledBuffer); 
      }
      //climb
      else if(solid == LEDConstants.PURPLE){ 
        for(var j = 0; j<ledBuffer.getLength(); j++){
          ledBuffer.setRGB(j, LEDConstants.PURPLE[0], LEDConstants.PURPLE[1], LEDConstants.PURPLE[2]);
        }
        led.setData(ledBuffer); 
      }
      
    }
    else{
      timer.stop(); 
    }
    System.out.println("Solids running"); 
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
