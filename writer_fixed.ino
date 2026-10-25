#include <Servo.h>
#include <avr/pgmspace.h>

// Assumes two 28BYJ-48 motors, each with a ULN2003 driver.
// Motor A is X; Motor B is Y.
const byte X_PINS[4] = {3, 4, 5, 6};
const byte Y_PINS[4] = {7, 8, 9, 10};
const byte SERVO_PIN = 2;

// Half-step sequence for a 28BYJ-48 + ULN2003.
const byte HALF_STEP[8][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// These are starting values. Calibrate for your mechanism.
const float X_STEPS_PER_MM = 40.0;
const float Y_STEPS_PER_MM = 40.0;

const float PIXEL_PITCH_MM = 2.5;
const int STEP_DELAY_US = 1200;

// Writing area in mm, measured from where the pen starts (the top-left corner).
// Letter paper is 215.9 x 279.4 mm; this leaves a 10 mm margin all round, so start the
// pen 10 mm in from the top-left corner. 0 = no limit.
// Text that would not fit is refused instead of driving the carriage into the frame.
const float PAPER_WIDTH_MM = 196;
const float PAPER_HEIGHT_MM = 259;

// Send this character from the page to abort the line being drawn.
const char STOP_CHAR = '!';

// Switch the motor coils off between lines so the motors and drivers stay cool
// and the Uno's USB supply is not loaded while idle.
const bool RELEASE_MOTORS_WHEN_IDLE = true;

// Adjust these angles so the pen lifts and touches gently.
const int PEN_UP_ANGLE = 70;
const int PEN_DOWN_ANGLE = 100;
const int PEN_SETTLE_MS = 250;   // time for the servo to finish moving

Servo penServo;
int xPhase = 0;
int yPhase = 0;
long xPosition = 0;
long yPosition = 0;
float cursorXmm = 0;
float cursorYmm = 0;
bool stopRequested = false;
bool penIsDown = true;   // unknown at power-up, so the first penUp() always moves the servo

// Each character is five columns wide. Bit 0 is the top row.
const byte FONT[26][5] PROGMEM = {
  {0x7E,0x11,0x11,0x11,0x7E}, // A
  {0x7F,0x49,0x49,0x49,0x36}, // B
  {0x3E,0x41,0x41,0x41,0x22}, // C
  {0x7F,0x41,0x41,0x22,0x1C}, // D
  {0x7F,0x49,0x49,0x49,0x41}, // E
  {0x7F,0x09,0x09,0x09,0x01}, // F
  {0x3E,0x41,0x49,0x49,0x7A}, // G
  {0x7F,0x08,0x08,0x08,0x7F}, // H
  {0x00,0x41,0x7F,0x41,0x00}, // I
  {0x20,0x40,0x41,0x3F,0x01}, // J
  {0x7F,0x08,0x14,0x22,0x41}, // K
  {0x7F,0x40,0x40,0x40,0x40}, // L
  {0x7F,0x02,0x0C,0x02,0x7F}, // M
  {0x7F,0x04,0x08,0x10,0x7F}, // N
  {0x3E,0x41,0x41,0x41,0x3E}, // O
  {0x7F,0x09,0x09,0x09,0x06}, // P
  {0x3E,0x41,0x51,0x21,0x5E}, // Q
  {0x7F,0x09,0x19,0x29,0x46}, // R
  {0x46,0x49,0x49,0x49,0x31}, // S
  {0x01,0x01,0x7F,0x01,0x01}, // T
  {0x3F,0x40,0x40,0x40,0x3F}, // U
  {0x1F,0x20,0x40,0x20,0x1F}, // V
  {0x3F,0x40,0x38,0x40,0x3F}, // W
  {0x63,0x14,0x08,0x14,0x63}, // X
  {0x07,0x08,0x70,0x08,0x07}, // Y
  {0x61,0x51,0x49,0x45,0x43}  // Z
};

void writeCoils(const byte pins[4], byte phase) {
  for (byte i = 0; i < 4; i++) {
    digitalWrite(pins[i], HALF_STEP[phase][i]);
  }
}

void stepX(int direction) {
  xPhase = (xPhase + direction + 8) % 8;
  writeCoils(X_PINS, xPhase);
  xPosition += direction;
}

void stepY(int direction) {
  yPhase = (yPhase + direction + 8) % 8;
  writeCoils(Y_PINS, yPhase);
  yPosition += direction;
}

// Move both axes together to absolute motor-step coordinates.
// Looks for the stop character the page sends when its Stop button is pressed.
// The page sends nothing else while a line is being drawn, so only the next byte is checked.
void pollStop() {
  if (Serial.available() > 0 && Serial.peek() == STOP_CHAR) {
    Serial.read();
    stopRequested = true;
  }
}

void moveTo(long targetX, long targetY) {
  long dx = targetX - xPosition;
  long dy = targetY - yPosition;

  long ax = labs(dx);
  long ay = labs(dy);
  long count = max(ax, ay);

  if (count == 0) return;

  int dirX = (dx >= 0) ? 1 : -1;
  int dirY = (dy >= 0) ? 1 : -1;
  long errX = 0;
  long errY = 0;

  for (long i = 0; i < count; i++) {
    pollStop();
    if (stopRequested) return;

    errX += ax;
    errY += ay;

    if (errX >= count) {
      stepX(dirX);
      errX -= count;
    }
    if (errY >= count) {
      stepY(dirY);
      errY -= count;
    }

    delayMicroseconds(STEP_DELAY_US);
  }
}

void moveToMM(float xMm, float yMm) {
  long xSteps = lround(xMm * X_STEPS_PER_MM);
  long ySteps = lround(yMm * Y_STEPS_PER_MM);
  moveTo(xSteps, ySteps);
}

void penUp() {
  if (!penIsDown) return;
  penServo.write(PEN_UP_ANGLE);
  delay(PEN_SETTLE_MS);
  penIsDown = false;
}

void penDown() {
  if (penIsDown) return;
  penServo.write(PEN_DOWN_ANGLE);
  delay(PEN_SETTLE_MS);
  penIsDown = true;
}

void releaseMotors() {
  for (byte i = 0; i < 4; i++) {
    digitalWrite(X_PINS[i], LOW);
    digitalWrite(Y_PINS[i], LOW);
  }
}

// Is the font pixel at (col,row) of this letter inked? Bit 0 of each column is the top row.
bool inkAt(byte letter, int col, int row) {
  if (col < 0 || col > 4 || row < 0 || row > 6) return false;
  return pgm_read_byte(&FONT[letter][col]) & (1 << row);
}

// Are the inked pixels (col,row) and (col+dc,row+dr) part of the same stroke?
// Side-by-side pixels always are. Diagonal pixels are only when neither of the two
// pixels beside them is inked (otherwise the diagonal would cut a corner).
bool linked(byte letter, int col, int row, int dc, int dr) {
  if (!inkAt(letter, col, row) || !inkAt(letter, col + dc, row + dr)) return false;
  if (dc != 0 && dr != 0) {
    return !inkAt(letter, col + dc, row) && !inkAt(letter, col, row + dr);
  }
  return true;
}

// Pen position in motor steps, used to avoid lifting the pen between joined strokes.
bool penIsAt(float xMm, float yMm) {
  return xPosition == lround(xMm * X_STEPS_PER_MM) && yPosition == lround(yMm * Y_STEPS_PER_MM);
}

// Draw a straight stroke from one font pixel to another (the pen stays down afterwards).
void stroke(int c0, int r0, int c1, int r1) {
  if (stopRequested) return;
  float x0 = cursorXmm + c0 * PIXEL_PITCH_MM;
  float y0 = cursorYmm + r0 * PIXEL_PITCH_MM;
  if (!penIsAt(x0, y0)) penUp();
  moveToMM(x0, y0);
  penDown();
  if (stopRequested) return;
  moveToMM(cursorXmm + c1 * PIXEL_PITCH_MM, cursorYmm + r1 * PIXEL_PITCH_MM);
}

void drawLetter(char c) {
  c = toupper(c);
  if (c < 'A' || c > 'Z') return;

  byte letter = c - 'A';

  // Strokes run right, down, down-right and down-left. A stroke starts at an inked
  // pixel that is not joined to the previous pixel in that direction.
  const int8_t DIRS[4][2] = { {1, 0}, {0, 1}, {1, 1}, {-1, 1} };

  for (byte d = 0; d < 4; d++) {
    int dc = DIRS[d][0];
    int dr = DIRS[d][1];

    for (int row = 0; row < 7; row++) {
      for (int col = 0; col < 5; col++) {
        if (!inkAt(letter, col, row)) continue;
        if (linked(letter, col - dc, row - dr, dc, dr)) continue;   // already part of a stroke

        int endCol = col;
        int endRow = row;
        while (linked(letter, endCol, endRow, dc, dr)) {
          endCol += dc;
          endRow += dr;
        }
        if (endCol != col || endRow != row) stroke(col, row, endCol, endRow);
      }
    }
  }

  // Pixels that are not joined to any neighbour become single dots.
  for (int row = 0; row < 7; row++) {
    for (int col = 0; col < 5; col++) {
      if (!inkAt(letter, col, row)) continue;

      bool joined = false;
      for (int dr = -1; dr <= 1 && !joined; dr++) {
        for (int dc = -1; dc <= 1 && !joined; dc++) {
          if ((dc != 0 || dr != 0) && linked(letter, col, row, dc, dr)) joined = true;
        }
      }
      if (!joined) stroke(col, row, col, row);
    }
  }

  penUp();
  cursorXmm += 6 * PIXEL_PITCH_MM;
}

const byte LINE_BUFFER_SIZE = 32;
char inputLine[LINE_BUFFER_SIZE];
byte inputLength = 0;
bool lineTooLong = false;

// Width of the current line in mm: letters use 6 pixels, spaces 3.
float lineWidthMm() {
  int pixels = 0;
  for (byte i = 0; i < inputLength; i++) {
    char c = toupper(inputLine[i]);
    if (c >= 'A' && c <= 'Z') pixels += 6;
    else if (c == ' ') pixels += 3;
  }
  return pixels * PIXEL_PITCH_MM;
}

void processLine() {
  if (lineTooLong) {
    Serial.println(F("Line too long. Send 31 characters or fewer."));
  } else if (PAPER_WIDTH_MM > 0 && lineWidthMm() > PAPER_WIDTH_MM) {
    Serial.println(F("Line too wide. Send fewer characters."));
  } else if (PAPER_HEIGHT_MM > 0 && cursorYmm + 6 * PIXEL_PITCH_MM > PAPER_HEIGHT_MM) {
    Serial.println(F("Out of paper. Load a new sheet and reset the Arduino."));
  } else {
    for (byte i = 0; i < inputLength && !stopRequested; i++) {
      char c = inputLine[i];

      if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';

      if (c >= 'A' && c <= 'Z') {
        drawLetter(c);
      } else if (c == ' ') {
        cursorXmm += 3 * PIXEL_PITCH_MM;
      }
    }

    // Start the next submitted line below this one.
    cursorXmm = 0;
    cursorYmm += 9 * PIXEL_PITCH_MM;
    penUp();
    if (RELEASE_MOTORS_WHEN_IDLE) releaseMotors();
    if (stopRequested) {
      stopRequested = false;
      releaseMotors();   // a stop always switches the motors off
      Serial.println(F("Stopped. Send another line."));
    } else {
      Serial.println(F("Done. Send another line."));
    }
  }

  inputLength = 0;
  lineTooLong = false;
}

void setup() {
  for (byte i = 0; i < 4; i++) {
    pinMode(X_PINS[i], OUTPUT);
    pinMode(Y_PINS[i], OUTPUT);
  }

  // Set the target angle before attaching so the pen does not jump to 90 degrees first.
  penServo.write(PEN_UP_ANGLE);
  penServo.attach(SERVO_PIN);
  delay(PEN_SETTLE_MS);
  penIsDown = false;

  Serial.begin(9600);
  Serial.println(F("Send A-Z; spaces advance; newline starts a new line."));
}

void loop() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Ignore carriage returns; process the line on newline.
    if (c == '\r') continue;

    // Stop character while idle (the line may have just finished): nothing to abort, but
    // answer so the page knows, and drop any half-received line.
    if (c == STOP_CHAR) {
      inputLength = 0;
      lineTooLong = false;
      penUp();
      releaseMotors();
      Serial.println(F("Stopped. Send another line."));
      continue;
    }

    if (c == '\n') {
      inputLine[inputLength] = '\0';
      processLine();
    } else if (inputLength < LINE_BUFFER_SIZE - 1) {
      inputLine[inputLength++] = c;
    } else {
      lineTooLong = true;
    }
  }
}