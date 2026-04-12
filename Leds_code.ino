#include <Adafruit_NeoPixel.h>

#define PIN        A2
#define NUMPIXELS  16

Adafruit_NeoPixel strip(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(50); 
  
  // Start with the default White
  setStripColor(strip.Color(255, 255, 255)); 
  
  Serial.begin(115200);
  while (!Serial); 
  Serial.println("Send U, H, or S to strobe. Any other key resets to White.");
}

void loop() {
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();

    // Ignore whitespace/newlines
    if (incomingByte == '\n' || incomingByte == '\r') return;

    if (incomingByte == 'U' || incomingByte == 'u') {
      strobeColor(strip.Color(0, 255, 0)); // Strobe Green
    } 
    else if (incomingByte == 'H' || incomingByte == 'h') {
      strobeColor(strip.Color(255, 0, 0)); // Strobe Red
    } 
    else if (incomingByte == 'S' || incomingByte == 's') {
      strobeColor(strip.Color(255, 255, 0)); // Strobe Yellow
    } 
    
    // Always return to White after a command (or if an unknown key is pressed)
    setStripColor(strip.Color(255, 255, 255));
  }
}

// Function to handle the 3-time strobe effect
void strobeColor(uint32_t color) {
  for (int i = 0; i < 12; i++) {
    setStripColor(color);           // Turn color ON
    delay(150);                     // Wait
    setStripColor(strip.Color(0,0,0)); // Turn OFF
    delay(150);                     // Wait
  }
}

// Function to set all pixels to a specific color
void setStripColor(uint32_t color) {
  for (int i = 0; i < NUMPIXELS; i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}