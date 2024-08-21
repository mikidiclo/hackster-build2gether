#include <ESP32Servo.h>
#include <Seeed_Arduino_SSCMA.h>

#define LEFT_RIGHT_PIN 2
#define UP_DOWN_PIN 1
#define DEF_HORIZONTAL_POS 50
#define DEF_VERTICAL_POS 60

int def_horizontal_pos = 50; // left_right_pos = 0 -> 100 
int def_vertical_pos = 60; // up_down_pos = 20 -> 100 

Servo left_right;
Servo up_down;

SSCMA model;

int current_pos = 50;

int encode_x(int x) {
  return (int)map(x, 0, 240, 80, -80);
}
int encode_y(int y) {
  return (int)map(y, 0, 240, 80, -80);
}

void horizontal_move(int new_x) {
  // move left or right
  if (abs(new_x) >= 4) {
    int new_pos = (int)map(new_x, -80, 80, 0, 180); // map from 0->100 degree to 20->200 px
    Serial.print("new_pos_x:");
    Serial.println(new_pos);
    
    left_right.write(new_pos);
    delay(100);
  }
}

void vertical_move(int new_y) {
  // move up or down
  if (abs(new_y) >= 2) {
    int new_pos = (int)map(new_y, -80, 80, 40, 160);
    Serial.print("new_pos_y:");
    Serial.println(new_pos);
    
    up_down.write(new_pos);
    delay(100);
  }
}

void setup() {
  left_right.attach(LEFT_RIGHT_PIN);
  up_down.attach(UP_DOWN_PIN);
  // left_right.write(def_horizontal_pos);
  // // up_down.write(def_vertical_pos);

  Serial.begin(115200);

  model.begin();
}

void loop() {
  // Invoke the model
  delay(20);
  
  if (!model.invoke()) {
    Serial.println("Invoke success");
    // If the bounding boxes are valid
    if (model.boxes().size() > 0) {
      // Get current axis
      int x = model.boxes()[0].x;
      int y = model.boxes()[0].y;

      // Debug
      Serial.print("x: ");
      Serial.print(x);
      Serial.print("\ny: ");
      Serial.print(y);
      Serial.print("\n");
      // Move to camera
      int new_x = encode_x(x);
      Serial.print("encode_x: ");
      Serial.print(new_x);
      Serial.print("\n");
      horizontal_move(new_x);
      
      int new_y = encode_y(x);
      Serial.print("encode_y: ");
      Serial.print(new_y);
      Serial.print("\n");
      vertical_move(new_y);
    }
  }

}
