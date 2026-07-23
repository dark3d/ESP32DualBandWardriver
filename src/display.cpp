#include "display.h"
#include <esp_system.h>
#include <math.h>

//Display::Display() {
//
//}

Display::Display(SPIClass* spi, int cs, int dc, int rst)
  : _spi(spi), _cs(cs), _dc(dc), _rst(rst) {
    tft = new Adafruit_ST7735(_spi, _cs, _dc, _rst);
}

void Display::begin() {
  pinMode(TFT_BL, OUTPUT);
  
  this->ctrlBacklight(false);

  //tft.init();
  #ifndef JCMK_HOST_BOARD
    tft->initR(INITR_MINI160x80_PLUGIN);
  #else
    tft->initR(INITR_MINI160x80);
  #endif

  tft->setSPISpeed(TFT_SPI_SPEED);

  this->clearScreen();
  
  tft->setTextWrap(false);

  tft->setRotation(3);

  if (esp_reset_reason() == ESP_RST_POWERON)
    this->kiloIntro();

  this->drawMonochromeImage160x80(logo2, 160, 80);

  // Full version (keep +dark3d so it never reads as koko firmware), centered
  // under the logo, above the boot bar. Full string only fits at size 1.
  tft->setTextSize(1);
  tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  String ver = FIRMWARE_VERSION;
  int16_t bx, by; uint16_t bw, bh;
  tft->getTextBounds(ver, 0, 0, &bx, &by, &bw, &bh);
  tft->setCursor((TFT_WIDTH - (int)bw) / 2, 64);
  tft->print(ver);

  this->ctrlBacklight(true);
}

void Display::kiloIntro() {
  this->ctrlBacklight(true);
  if (esp_random() & 1) this->introCopper();
  else                  this->introMatrix();
}

// --- Scene 1: copper raster bars + sine-wave KILO + greetz scroller ---
void Display::introCopper() {
  const char* scroll =
    "   DARK3D GREETZ ::  THANKS FOR ALL THE TESTING  ::  KILOGRAMOWY  ::  "
    "ESP32 BUILDER . ASIC ENTHUSIAST . WDG EXPLORER   ";
  const char* logo = "KILO";
  int slen = (int)strlen(scroll);

  const int frames = 120;
  for (int f = 0; f < frames; f++) {
    float t = f * 0.16f;

    for (int y = 0; y < TFT_HEIGHT; y += 4) {
      float a = y * 0.11f - t * 1.4f;
      uint8_t r = (uint8_t)(12 + 14 * (sinf(a) + 1.0f));
      uint8_t g = (uint8_t)(30 + 55 * (sinf(a + 2.1f) + 1.0f));
      uint8_t b = (uint8_t)(45 + 50 * (sinf(a + 4.2f) + 1.0f));
      uint16_t c = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3);
      tft->fillRect(0, y, TFT_WIDTH, 4, c);
    }

    tft->setTextSize(3);
    int base_x = (TFT_WIDTH - 4 * 18) / 2;
    for (int i = 0; i < 4; i++) {
      int gx = base_x + i * 18;
      int gy = 12 + (int)(9.0f * sinf(t * 1.3f + i * 0.9f));
      tft->setTextColor(ST77XX_BLACK);
      tft->setCursor(gx + 2, gy + 2);
      tft->write(logo[i]);
      tft->setTextColor(0x07FF);
      tft->setCursor(gx, gy);
      tft->write(logo[i]);
    }

    tft->setTextSize(2);
    tft->setTextColor(ST77XX_YELLOW);
    int sx = TFT_WIDTH - f * 7;
    for (int i = 0; i < slen; i++) {
      int cx = sx + i * 12;
      if (cx < -12 || cx > TFT_WIDTH) continue;
      int cy = 58 + (int)(3.0f * sinf(cx * 0.045f + t));
      tft->setCursor(cx, cy);
      tft->write(scroll[i]);
    }

    delay(22);
  }
}

// 'B' with two vertical strokes = a scalable Bitcoin mark
void Display::drawBtcMark(GFXcanvas16* cv, int x, int y, int s, uint16_t col) {
  cv->setTextSize(s);
  cv->setTextColor(col);
  cv->setCursor(x, y);
  cv->write('B');
  int bw = (s >= 3) ? 3 : 2;
  int bh = 8 * s + s;
  int by = y - s / 2;
  cv->fillRect(x + 2 * s - 1, by, bw, bh, col);
  cv->fillRect(x + 4 * s,     by, bw, bh, col);
}

// --- Scene 3: Bitcoin hash-rain + flowing ₿ marks + gold KILO (no flicker) ---
void Display::introMatrix() {
  GFXcanvas16 cv(TFT_WIDTH, TFT_HEIGHT);
  const uint16_t GOLD    = 0xFD20;   // bright Bitcoin amber (KILO)
  const uint16_t DIMGOLD = 0xC280;   // dimmer amber (flowing marks)
  const int COLS = 27;
  float head[COLS];
  uint8_t spd[COLS];
  for (int c = 0; c < COLS; c++) {
    head[c] = random(-40, 80);
    spd[c] = random(2, 6);
  }
  const char* hex = "0123456789ABCDEF";

  // flowing Bitcoin marks: two large, one medium (medium speed)
  const int NB = 3;
  int   bs_[NB]  = { 3, 3, 2 };
  int   bx_[NB]  = { 8, 118, 64 };
  float by_[NB]  = { (float)random(-50, 0), (float)random(-70, -20), (float)random(-30, 20) };
  float bsp_[NB] = { 0.7f, 0.95f, 1.5f };

  const int frames = 135;
  for (int f = 0; f < frames; f++) {
    cv.fillScreen(ST77XX_BLACK);

    // hex hash rain
    cv.setTextSize(1);
    for (int c = 0; c < COLS; c++) {
      head[c] += spd[c] * 0.5f;
      if (head[c] > 92) { head[c] = random(-40, 0); spd[c] = random(2, 6); }
      int hx = c * 6;
      for (int k = 0; k < 8; k++) {
        int hy = (int)head[c] - k * 8;
        if (hy < 0 || hy >= TFT_HEIGHT) continue;
        uint16_t col;
        if (k == 0) col = 0xFFFF;
        else { int r = 31 - k; if (r < 8) r = 8;
               int g = 34 - k * 4; if (g < 4) g = 4;
               col = (uint16_t)((r << 11) | (g << 5)); }
        cv.setTextColor(col);
        cv.setCursor(hx, hy);
        cv.write(hex[random(0, 16)]);
      }
    }

    // flowing Bitcoin marks (over the rain)
    for (int j = 0; j < NB; j++) {
      by_[j] += bsp_[j];
      if (by_[j] > TFT_HEIGHT + 4) {
        by_[j] = -(float)(8 * bs_[j]);
        bx_[j] = random(0, TFT_WIDTH - 6 * bs_[j]);
      }
      this->drawBtcMark(&cv, bx_[j], (int)by_[j], bs_[j], DIMGOLD);
    }

    // static ₿ mark, top-center
    this->drawBtcMark(&cv, TFT_WIDTH / 2 - 6, 0, 2, GOLD);

    // KILO, bright gold, boxed so it reads over everything
    cv.setTextSize(3);
    int bx = (TFT_WIDTH - 4 * 18) / 2;
    cv.fillRect(bx - 4, 30 - 4, 4 * 18 + 8, 24 + 8, ST77XX_BLACK);
    for (int i = 0; i < 4; i++) {
      cv.setTextColor(GOLD);
      cv.setCursor(bx + i * 18, 30);
      cv.write("KILO"[i]);
    }

    tft->drawRGBBitmap(0, 0, cv.getBuffer(), TFT_WIDTH, TFT_HEIGHT);
    delay(12);
  }
}

void Display::drawCenteredText(String text, bool centerVertically) {
  tft->setRotation(3);  // Landscape
  tft->setTextSize(1);  // 6x8 per char
  tft->setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft->setTextWrap(false);

  uint8_t charWidth = 6;
  uint8_t charHeight = 8;

  uint16_t textWidth = text.length() * charWidth;
  uint16_t textHeight = charHeight;

  uint16_t x = (TFT_WIDTH - textWidth) / 2;
  uint16_t y = centerVertically ? (TFT_HEIGHT - textHeight) / 2 : tft->getCursorY();

  tft->setCursor(x, y);
  tft->print(text);
}

// https://javl.github.io/image2cpp/
void Display::drawMonochromeImage160x80(const uint8_t* imageData, int width, int height) {
  tft->startWrite();

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int byteIndex = (y * (width / 8)) + (x / 8);
      uint8_t byteVal = pgm_read_byte(&imageData[byteIndex]);

      // MSB first (bit 7 is leftmost pixel)
      bool pixelOn = (byteVal >> (7 - (x % 8))) & 0x01;
      // Vertical gradient on the logo: cyan (top) -> magenta (bottom). RGB565.
      uint16_t color = ST77XX_BLACK;
      if (pixelOn) {
        int denom = (height > 1) ? (height - 1) : 1;
        int r = (31 * y) / denom;        // 0 -> 31
        int g = 63 - (63 * y) / denom;   // 63 -> 0
        int b = 31;                      // constant
        color = (uint16_t)((r << 11) | (g << 5) | b);
      }

      // Adjust for rotation 3 (landscape)
      int x_rot = x;
      int y_rot = y;

      tft->writePixel(x_rot, y_rot, color);
    }
  }

  tft->endWrite();
}

void Display::ctrlBacklight(bool on) {
  if (on)
    digitalWrite(TFT_BL, ON);
  else
    digitalWrite(TFT_BL, OFF);
}

void Display::clearScreen() {
  tft->fillScreen(ST77XX_BLACK);
}

void Display::main(uint32_t currentTime) {

}