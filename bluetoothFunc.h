void resetBT() {
  digitalWrite(BT_PIO4, LOW);
  delay(100);
  digitalWrite(BT_PIO4, HIGH);
  delay(1100);
}

void BT_discoveryON()
{
  Serial.println("bt_discovery_on");
  resetBT();
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  Serial1.print("");
  delay(50);
//  Serial.print("Response: ");
//  Serial.println(Serial1.read());
  delay(100);
  Serial1.println("SQ,0");
  delay(50);
//  Serial.print("Response: ");
//  Serial.println(Serial1.read());
  delay(100);
  Serial1.println("---");
  delay(50);
//  Serial.print("Response: ");
//  Serial.println(Serial1.read());
  delay(100);
  resetBT();
}

void BT_discoveryOFF()
{
  Serial.println("bt_discovery_off");
  resetBT();
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  Serial1.print("");
  delay(50);
//  Serial.print("Response: ");
//  Serial.println(Serial1.read());
  delay(100);  // Short delay, wait for the Mate to send back CMD
  Serial1.println("SQ,8"); // Quiet, Turn off Discovery and Connectability
  delay(50);
  Serial1.println ("R,1"); // Restart BT
  delay(50);
}

void BT_reconnect()
{
  Serial.println("bt_reconnect");
  resetBT();
  delay(100);
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  Serial1.print("");
  delay(50);
//  Serial.print("Response: ");
//  Serial.println(Serial1.read());
  delay(100);  // Short delay, wait for the Mate to send back CMD
  Serial1.println("CFI"); // Connects to previous connected device
  delay(100);
  Serial1.println("---");
  delay(100);
//  Serial.print("Response2: ");
//  Serial.println(Serial1.read());
  delay(100);
}

void BT_disconnect()
{
  Serial.println("bt_disconnect");
  resetBT();
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  Serial1.print("");
  delay(30);
//  Serial.print("Response: ");
//  Serial.println(Serial1.read());
  delay(100);
  Serial1.println ("K,"); // Disconnects from active device
  delay(50);
  Serial1.println("---");
  delay(50);
}
