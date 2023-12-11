// This Code waits for the the object to be covered, then runs one task and waits for the object to be revealed to do another task.
  
#include <Pixy2.h>

// This is the main Pixy object 
Pixy2 pixy;

void setup()
{
  Serial.begin(115200);
  Serial.print("Starting...\n");
  
  pixy.init();
}

void loop()
{ 
  pixy.ccc.getBlocks(); //Gets information from the pixy cam to see if an object has been detected

while (pixy.ccc.numBlocks) { //If an object is detected (true), get information from the pixy cam and check for an object again
  pixy.ccc.getBlocks();
}
//Once the object is no longer detected, the while loop ends and the text is printed.
Serial.println("Turn on Lamp and Grab Pen");
//Check information on the pixy cam again
pixy.ccc.getBlocks();
while (!(pixy.ccc.numBlocks)) { //While nothing is detected (false), keep checking
  pixy.ccc.getBlocks();
}
//Once the object is revealed again, print the text. The overall loop then causes this code to repeat.
Serial.println("Turn off lamp and Return Pen");
}
