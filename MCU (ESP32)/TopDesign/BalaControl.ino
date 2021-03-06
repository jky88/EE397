/* Copyright (C) 2018 Charles. All rights reserved.

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation; either version 2 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*************************************************************************/
// Task0 :                Self-balance Control
/*************************************************************************/

void balaControl(void *parameter)
{
  // Initialize bala ...
  myBala.begin();
  delay(500);

  // Update PID parameters ...
  if (myFlash.initEEPROM(myBala))
  {
    Serial.println("Restart ...");
    ESP.restart();
  }

  while(1)
  {
    static uint32_t control_interval = millis() + 5;
    if (millis() > control_interval) 
    {
      control_interval = millis() + 5;
      myBala.run();
    }  
    vTaskDelay(10); 
  }
  vTaskDelete(NULL);  
}
