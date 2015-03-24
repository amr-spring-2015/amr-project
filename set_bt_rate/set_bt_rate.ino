/*
 * echo - echo characters back to bluetooth device
 *
 * Waits for a connection and then echos each charater received.
 *
 * Debugging is enabled, so if you open the 'Serial Monitor' you can see
 * the search for the HC05 baud and the wait for the BT connection.
 */
#include <Arduino.h>
#include <HardwareSerial.h>

int ledpin=13;
int D3 = 3;
const int LINE_BUFFER_SIZE = 80;

void setup()
{
  Serial.begin(38400);
  pinMode(ledpin,OUTPUT);
  pinMode(D3,OUTPUT);
  digitalWrite(D3,1);
}

int read_line(char* buffer, int bufsize)
{
  for (int index = 0; index < bufsize; index++) {
    // Wait until characters are available
    while (Serial.available() == 0) {
    }

    char ch = Serial.read(); // read next character
    //Serial.print(ch); // echo it back: useful with the serial monitor (optional)

    if (ch == '\n') {
      buffer[index] = 0; // end of line reached: null terminate string
      return index; // success: return length of string (zero if string is empty)
    }

    buffer[index] = ch; // Append character to buffer
  }

  // Reached end of buffer, but have not seen the end-of-line yet.
  // Discard the rest of the line (safer than returning a partial line).

  char ch;
  do {
    // Wait until characters are available
    while (Serial.available() == 0) {
    }
    ch = Serial.read(); // read next character (and discard it)
    //Serial.print(ch); // echo it back
  } while (ch != '\n');

  buffer[0] = 0; // set buffer to empty string even though it should not be used
  return -1; // error: return negative one to indicate the input was too long
}


void loop()
{ 
  char line[LINE_BUFFER_SIZE];
  //if (Serial.available()){
    Serial.println("AT+UART=460800,0,0");
    //delay(1000);
    read_line(line, sizeof(line));
    if (line=="OK"){
      digitalWrite(ledpin,1);
    }
    Serial.println("AT+UART?");
    //delay(1000);
    read_line(line, sizeof(line));    
    if (line=="+UART:230400,0,0"){
      digitalWrite(ledpin,0);
    }
  //}
    return;
}
