// Import the Processing Serial library
import processing.serial.*;

// Declare a Serial object
Serial myPort;


PrintWriter output;


// Timer variables for periodic flushing
long lastFlushTime = 0;
final long FLUSH_INTERVAL_MS = 1000; // Flush every 1 second
String filePath = "continuous_output.csv";


// Variables to store the incoming data
int xPos = 0;  
int yPos = 0;
int intensity = 0;
int total_lines = 0;

// HORIZ 50000 30 degrees left
// HORIZ 40600 straight
// HORIZ 32000 30 degrees left


//VERT 24700 = level
//VERT 50000 = 30 degrees down
//VERT 2000 = 30 degrees up


final int HORIZ_LEFT  = 47000;
final int HORIZ_RIGHT = 35000;

final int VERT_BOTTOM = 55000;
final int VERT_TOP     = 20000;

final int COUNTS_BASELINE = 0;
final int COUNTS_MAX = 250;
final float COUNTS_GAMMA = 1;

final int SCALED_WIDTH = 800;
final int SCALED_HEIGHT = 800;



void setup() {
  // Set the size of the window. Adjust as needed for your display.
  // Using 600x400 as an example, assuming X and Y values fit within this range.
  size(800, 800);
  background(128); // Set initial background to black

  try {
    // Open the PrintWriter once in setup().
    // createWriter() will create the file if it doesn't exist, or overwrite it if it does.
    output = createWriter(filePath);
    println("Opened CSV file for continuous writing: " + filePath);
    
    // Initialize the lastFlushTime
    lastFlushTime = millis();

  } catch (Exception e) {
    // Catch any exceptions that might occur during file opening (e.g., permissions issues).
    println("Error opening CSV file: " + e.getMessage());
    exit(); // Exit the sketch if the file cannot be opened
  }
  
  

  // Print a list of all available serial ports to the console.
  // This is helpful for identifying the correct port for your device.
  println(Serial.list());

  // Open the port that the device is connected to.
  // On Windows, this might be "COM3", "COM4", etc.
  // You might need to change 'Serial.list()[0]' to 'Serial.list()[1]' or
  // specify the port directly like 'new Serial(this, "COM3", 9600);'
  // if your device is not on the first port in the list.
  String portName = Serial.list()[0]; // Try the first available port by default
  
  // Attempt to find a common Windows serial port name if the first one doesn't work
  if (Serial.list().length > 0) {
    for (String p : Serial.list()) {
      if (p.contains("COM")) { // Look for COM ports on Windows
        portName = p;
        break;
      }
    }
  } else {
    println("No serial ports found. Please check your connections.");
    exit(); // Exit if no ports are found
  }

  try {
    myPort = new Serial(this, portName, 9600); // Open the port at 9600 baud rate
    myPort.bufferUntil('\n'); // Buffer data until a newline character is received
    println("Opened serial port: " + portName);
  } catch (Exception e) {
    println("Error opening serial port " + portName + ": " + e.getMessage());
    println("Please check if the port is correct and not in use by another program.");
    exit(); // Exit if there's an error opening the port
  }
}

void draw() {
  // The draw loop runs continuously.
  // We clear the background here to see individual pixels as they arrive.
  // If you want to accumulate pixels (like drawing a trail), remove this line.
  // background(0); // Uncomment this line if you want to clear the screen every frame
}

// This function is called automatically by Processing when new data arrives
// on the serial port, specifically when a newline character is received
// because of myPort.bufferUntil('\n') in setup().
void serialEvent(Serial p) {
  try {
    // Read the incoming string until the newline character
    String inString = p.readStringUntil('\n');

    // Check if the string is not null and not empty after trimming whitespace
    if (inString != null) {
      inString = trim(inString); // Remove leading/trailing whitespace

      // Split the string into an array of strings using the comma as a delimiter
      String[] data = splitTokens(inString, ",");

      // Ensure we have exactly three parts (X, Y, Intensity)
      if (data.length == 3) {
        // Parse the strings to integers
        xPos = (int)map(float(data[0]),HORIZ_LEFT, HORIZ_RIGHT, 0, SCALED_WIDTH);
        yPos = (int)map(float(data[1]),VERT_TOP, VERT_BOTTOM, 0, SCALED_HEIGHT);
        float raw_intensity = map(float(data[2])-COUNTS_BASELINE,0, COUNTS_MAX, 0, 1);
        float gamma_corrected = pow(raw_intensity, COUNTS_GAMMA);
  
  
        // Constrain values to be within the window dimensions and color range
        xPos = constrain(xPos, 0, width - 1);
        yPos = constrain(yPos, 0, height - 1);
        intensity = (int)map(gamma_corrected, 0, 1, 0, 255); // Intensity for grayscale
        intensity = constrain(intensity, 0, 255);
        // Set the pixel color at the received X, Y coordinates
        // The color is grayscale, so intensity is used for R, G, and B
        //stroke(intensity); // Set the drawing color to grayscale
        //point(xPos, yPos); // Draw a point at the specified coordinates
        
        // You can also use set() for direct pixel manipulation:
         color c = color(intensity); // Create a grayscale color
         set(xPos, yPos, c); // Set the pixel at (xPos, yPos) to color c
         
        output.println(xPos + "," + yPos + "," + intensity);

        // Print the received data to the console for debugging
        println("Received: X=" + xPos + ", Y=" + yPos + ", Intensity=" + intensity + ", Lines=" + total_lines++);
      } else {
        println("Received malformed data (expected 3 comma-separated values): " + inString);
      }
    }
  } catch (NumberFormatException e) {
    println("Error parsing numbers from serial data: " + e.getMessage());
  } catch (Exception e) {
    println("An unexpected error occurred in serialEvent: " + e.getMessage());
  }
  
  
  // Periodically flush the buffer to ensure data is written to disk.
  // This prevents data loss if the program crashes before stop() is called.
  if (millis() - lastFlushTime > FLUSH_INTERVAL_MS) {
    output.flush();
    //println("CSV file flushed.");
    lastFlushTime = millis();
  }
  
  
}

// Optional: If you want to handle window closing gracefully
void stop() {
  if (myPort != null) {
    myPort.stop(); // Close the serial port when the sketch stops
    println("Serial port closed.");
  }
  super.stop();
}
