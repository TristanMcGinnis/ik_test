
#include <FABRIK2D.h>

int lengths[] = {468, 378, 97}; // 3DOF arm where shoulder to elbow is 225mm, elbow to wrist is 150mm and wrist to end effector is 100mm.
Fabrik2D fabrik2D(4, lengths); // This arm has 4 joints; one in the origin, the elbow, the wrist and the end effector.

void setup() {
  Serial.begin(9600);

  fabrik2D.setTolerance(1.0);
}

void loop() {

  float target_x = 200.0;
  float target_y = 200.0;
  int step = 0;
  while(1)
  {
    fabrik2D.solve(target_x,target_y, -M_PI/4.0, lengths);
    Serial.print(fabrik2D.getAngle(0)* 57296 / 1000);
    Serial.print(",");
    Serial.print(fabrik2D.getAngle(1)* 57296 / 1000);
    Serial.print(",");
    Serial.print(fabrik2D.getAngle(2)* 57296 / 1000);
    Serial.println();

    delay(175);

    //Move the end effector in a specific pattern.
    //Right 200, down 200, up 200, left 200, repeat.
    if(step == 0)
    {
      target_x += 20.0;
      if(target_x == 400.0)
      {
        step = 1;
      }
    }
    else if(step == 1)
    {
      target_y -= 20.0;
      if(target_y == 0.0)
      {
        step = 2;
      }
    }
    else if(step == 2)
    {
      target_y += 20.0;
      if(target_y == 200.0)
      {
        step = 3;
      }
    }
    else if(step == 3)
    {
      target_x -= 20.0;
      if(target_x == 200.0)
      {
        step = 0;
      }
    }
    
  }


}
