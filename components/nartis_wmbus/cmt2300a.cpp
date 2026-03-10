#include "cmt2300a.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome::nartis_wmbus {

static const char *const TAG = "nartis_wmbus.cmt2300a";

// ============================================================================
// Bit-banged 3-wire SPI: MSB first
// Protocol: CSB low -> 8-bit addr (bit7=0 for write, 1 for read) -> 8-bit data -> CSB high
// ============================================================================

void CMT2300A::spi_write_byte_(uint8_t byte) {
  for (int8_t i = 7; i >= 0; i--) {
    this->pin_sclk_->digital_write(false);
    this->pin_sdio_->digital_write((byte >> i) & 1);
    delayMicroseconds(2);
    this->pin_sclk_->digital_write(true);
    delayMicroseconds(2);
  }
  this->pin_sclk_->digital_write(false);
}

uint8_t CMT2300A::spi_read_byte_() {
  uint8_t byte = 0;
  this->pin_sdio_->pin_mode(gpio::FLAG_INPUT);
  for (int8_t i = 7; i >= 0; i--) {
    this->pin_sclk_->digital_write(false);
    delayMicroseconds(2);
    this->pin_sclk_->digital_write(true);
    if (this->pin_sdio_->digital_read())
      byte |= (1 << i);
    delayMicroseconds(2);
  }
  this->pin_sclk_->digital_write(false);
  this->pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  return byte;
}

void CMT2300A::write_reg(uint8_t addr, uint8_t value) {
  this->pin_csb_->digital_write(false);
  delayMicroseconds(2);
  this->spi_write_byte_(addr & 0x7F);  // bit7 = 0 for write (per datasheet & firmware)
  this->spi_write_byte_(value);
  this->pin_csb_->digital_write(true);
  delayMicroseconds(2);
}

uint8_t CMT2300A::read_reg(uint8_t addr) {
  this->pin_csb_->digital_write(false);
  delayMicroseconds(2);
  this->spi_write_byte_(addr | 0x80);  // bit7 = 1 for read (per datasheet & firmware)
  uint8_t value = this->spi_read_byte_();
  this->pin_csb_->digital_write(true);
  delayMicroseconds(2);
  return value;
}

void CMT2300A::write_bank(uint8_t start_reg, const uint8_t *data, uint8_t count) {
  for (uint8_t i = 0; i < count; i++) {
    this->write_reg(start_reg + i, data[i]);
  }
}

void CMT2300A::write_fifo(const uint8_t *data, uint16_t len) {
  this->pin_fcsb_->digital_write(false);
  delayMicroseconds(1);
  for (uint16_t i = 0; i < len; i++) {
    this->spi_write_byte_(data[i]);
  }
  this->pin_fcsb_->digital_write(true);
  delayMicroseconds(1);
}

void CMT2300A::read_fifo(uint8_t *data, uint16_t len) {
  this->pin_fcsb_->digital_write(false);
  delayMicroseconds(1);
  this->pin_sdio_->pin_mode(gpio::FLAG_INPUT);
  for (uint16_t i = 0; i < len; i++) {
    data[i] = 0;
    for (int8_t bit = 7; bit >= 0; bit--) {
      this->pin_sclk_->digital_write(false);
      delayMicroseconds(1);
      this->pin_sclk_->digital_write(true);
      if (this->pin_sdio_->digital_read())
        data[i] |= (1 << bit);
      delayMicroseconds(1);
    }
    this->pin_sclk_->digital_write(false);
  }
  this->pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_fcsb_->digital_write(true);
  delayMicroseconds(1);
}

// ============================================================================
// Register Configuration
// ============================================================================

void CMT2300A::write_config_(const uint8_t *config, uint8_t channel) {
  uint8_t offset = 0;
  uint8_t ch = (channel < 4) ? channel : 1;
  uint8_t tmp;

  ESP_LOGV(TAG, "write_config_: ch=%d (effective=%d), config=%s", channel, ch,
           config == CMT2300A_TX_CONFIG ? "TX" : config == CMT2300A_RX_CONFIG ? "RX" : "unknown");

  // Pre-config: firmware function 0x13FEA does these before writing banks:
  // RMW on CUS_MODE_STA: clear RSTN_IN_EN, set CFG_RETAIN
  tmp = this->read_reg(CMT2300A_CUS_MODE_STA);
  ESP_LOGVV(TAG, "write_config_: pre-config CUS_MODE_STA=0x%02X -> 0x%02X", tmp,
            (tmp & ~CMT2300A_MASK_RSTN_IN_EN) | CMT2300A_MASK_CFG_RETAIN);
  this->write_reg(CMT2300A_CUS_MODE_STA, (tmp & ~CMT2300A_MASK_RSTN_IN_EN) | CMT2300A_MASK_CFG_RETAIN);
  // RMW on CUS_EN_CTL: set ERROR_STOP_EN
  tmp = this->read_reg(CMT2300A_CUS_EN_CTL);
  ESP_LOGVV(TAG, "write_config_: pre-config CUS_EN_CTL=0x%02X -> 0x%02X", tmp, tmp | CMT2300A_MASK_ERROR_STOP_EN);
  this->write_reg(CMT2300A_CUS_EN_CTL, tmp | CMT2300A_MASK_ERROR_STOP_EN);

  // Write 6 register banks in order
  ESP_LOGVV(TAG, "write_config_: bank CMT (0x%02X, %d regs)", CMT2300A_CMT_BANK_ADDR, CMT_BANK_CUS_CMT);
  this->write_bank(CMT2300A_CMT_BANK_ADDR, &config[offset], CMT_BANK_CUS_CMT);
  offset += CMT_BANK_CUS_CMT;

  ESP_LOGVV(TAG, "write_config_: bank SYS (0x%02X, %d regs)", CMT2300A_SYSTEM_BANK_ADDR, CMT_BANK_CUS_SYS);
  this->write_bank(CMT2300A_SYSTEM_BANK_ADDR, &config[offset], CMT_BANK_CUS_SYS);
  offset += CMT_BANK_CUS_SYS;

  // Use channel table for CUS_FREQ instead of default from config
  ESP_LOGVV(TAG, "write_config_: bank FREQ (0x%02X, %d regs) — channel %d = %.3f MHz",
            CMT2300A_FREQUENCY_BANK_ADDR, CMT_BANK_CUS_FREQ, ch, CMT2300A_FREQ_MHZ[ch]);
  this->write_bank(CMT2300A_FREQUENCY_BANK_ADDR, CMT2300A_FREQ_CHANNELS[ch], CMT_BANK_CUS_FREQ);
  offset += CMT_BANK_CUS_FREQ;

  ESP_LOGVV(TAG, "write_config_: bank DATA_RATE (0x%02X, %d regs)", CMT2300A_DATA_RATE_BANK_ADDR, CMT_BANK_CUS_DATA_RATE);
  this->write_bank(CMT2300A_DATA_RATE_BANK_ADDR, &config[offset], CMT_BANK_CUS_DATA_RATE);
  offset += CMT_BANK_CUS_DATA_RATE;

  ESP_LOGVV(TAG, "write_config_: bank BASEBAND (0x%02X, %d regs)", CMT2300A_BASEBAND_BANK_ADDR, CMT_BANK_CUS_BASEBAND);
  this->write_bank(CMT2300A_BASEBAND_BANK_ADDR, &config[offset], CMT_BANK_CUS_BASEBAND);
  offset += CMT_BANK_CUS_BASEBAND;

  ESP_LOGVV(TAG, "write_config_: bank TX (0x%02X, %d regs)", CMT2300A_TX_BANK_ADDR, CMT_BANK_CUS_TX);
  this->write_bank(CMT2300A_TX_BANK_ADDR, &config[offset], CMT_BANK_CUS_TX);

  // Post-bank fixup: CUS_CMT10 — clear bits 2:0, set bit 1
  // Firmware does this at end of both TX and RX config functions
  tmp = this->read_reg(CMT2300A_CUS_CMT10);
  ESP_LOGVV(TAG, "write_config_: CUS_CMT10 fixup 0x%02X -> 0x%02X", tmp, (tmp & 0xF8) | 0x02);
  this->write_reg(CMT2300A_CUS_CMT10, (tmp & 0xF8) | 0x02);

  // Apply post-config fixups (firmware function 0x13666)
  ESP_LOGVV(TAG, "write_config_: applying post-config fixups");
  this->apply_fixups_();
  ESP_LOGV(TAG, "write_config_: done");
}

void CMT2300A::apply_fixups_() {
  // Post-config register fixups — firmware function 0x13666.
  // Control Bank 1 (0x60-0x6A) is never bulk-written; always individual RMW.
  // SDK names from cmt2300a_defs.h. See decompiled/cmt2300a_fixups.c.

  uint8_t tmp;

  // 1: CUS_IO_SEL — GPIO pin mode: GPIO2=INT1, GPIO3=INT2, GPIO1=DOUT/DIN
  uint8_t io_sel_val = CMT2300A_GPIO3_SEL_INT2 | CMT2300A_GPIO2_SEL_INT1;
  ESP_LOGVV(TAG, "fixup 1: CUS_IO_SEL = 0x%02X (GPIO2=INT1, GPIO3=INT2)", io_sel_val);
  this->write_reg(CMT2300A_CUS_IO_SEL, io_sel_val);

  // 2: CUS_INT2_CTL — preserve top 3 bits, set INT2_SEL = RX_FIFO_TH
  tmp = this->read_reg(CMT2300A_CUS_INT2_CTL);
  ESP_LOGVV(TAG, "fixup 2: CUS_INT2_CTL 0x%02X -> 0x%02X", tmp, (tmp & 0xE0) | CMT2300A_INT_SEL_RX_FIFO_TH);
  this->write_reg(CMT2300A_CUS_INT2_CTL, (tmp & 0xE0) | CMT2300A_INT_SEL_RX_FIFO_TH);

  // 3: CUS_INT_EN = TX_DONE + PREAM_OK + SYNC_OK + PKT_DONE
  uint8_t int_en_val = CMT2300A_MASK_TX_DONE_EN | CMT2300A_MASK_PREAM_OK_EN | CMT2300A_MASK_SYNC_OK_EN |
                       CMT2300A_MASK_PKT_DONE_EN;
  ESP_LOGVV(TAG, "fixup 3: CUS_INT_EN = 0x%02X", int_en_val);
  this->write_reg(CMT2300A_CUS_INT_EN, int_en_val);

  // 4: CUS_SYS2 — clear bits 7:5 (disable LFOSC calibration timers)
  tmp = this->read_reg(CMT2300A_CUS_SYS2);
  ESP_LOGVV(TAG, "fixup 4: CUS_SYS2 0x%02X -> 0x%02X", tmp,
            tmp & ~(CMT2300A_MASK_LFOSC_RECAL_EN | CMT2300A_MASK_LFOSC_CAL1_EN | CMT2300A_MASK_LFOSC_CAL2_EN));
  this->write_reg(CMT2300A_CUS_SYS2,
                  tmp & ~(CMT2300A_MASK_LFOSC_RECAL_EN | CMT2300A_MASK_LFOSC_CAL1_EN | CMT2300A_MASK_LFOSC_CAL2_EN));

  // 5: CUS_FIFO_CTL — set FIFO_MERGE_EN (merge TX+RX → 64 bytes)
  tmp = this->read_reg(CMT2300A_CUS_FIFO_CTL);
  ESP_LOGVV(TAG, "fixup 5: CUS_FIFO_CTL 0x%02X -> 0x%02X", tmp, tmp | CMT2300A_MASK_FIFO_MERGE_EN);
  this->write_reg(CMT2300A_CUS_FIFO_CTL, tmp | CMT2300A_MASK_FIFO_MERGE_EN);

  // 6: CUS_PKT29 — preserve FIFO_AUTO_RES_EN, set FIFO threshold to 15
  tmp = this->read_reg(CMT2300A_CUS_PKT29);
  ESP_LOGVV(TAG, "fixup 6: CUS_PKT29 0x%02X -> 0x%02X", tmp, (tmp & CMT2300A_MASK_FIFO_AUTO_RES_EN) | 0x0F);
  this->write_reg(CMT2300A_CUS_PKT29, (tmp & CMT2300A_MASK_FIFO_AUTO_RES_EN) | 0x0F);

  // 7: CUS_SYS11 — preserve top 3, set low 5 to 0x12 (RSSI config)
  tmp = this->read_reg(CMT2300A_CUS_SYS11);
  ESP_LOGVV(TAG, "fixup 7: CUS_SYS11 0x%02X -> 0x%02X", tmp, (tmp & 0xE0) | 0x12);
  this->write_reg(CMT2300A_CUS_SYS11, (tmp & 0xE0) | 0x12);

  // 8a: TX power level=0 — CUS_CMT4 + CUS_TX8/TX9 (PA config)
  ESP_LOGVV(TAG, "fixup 8a: TX power — CMT4=0x1C TX8=0x10 TX9=0x02");
  this->write_reg(CMT2300A_CUS_CMT4, 0x1C);
  this->write_reg(CMT2300A_CUS_TX8, 0x10);
  this->write_reg(CMT2300A_CUS_TX9, 0x02);

  // 8b: Freq config=0 — CDR/AFC tuning (PKT10-PKT13 in Baseband Bank)
  ESP_LOGVV(TAG, "fixup 8b: CDR/AFC — PKT10=0x8D PKT11=0xF6 PKT12=0x55 PKT13=0x55");
  this->write_reg(CMT2300A_CUS_PKT10, 0x8D);
  this->write_reg(CMT2300A_CUS_PKT11, 0xF6);
  this->write_reg(CMT2300A_CUS_PKT12, 0x55);
  this->write_reg(CMT2300A_CUS_PKT13, 0x55);

  // 8d: CUS_INT1_CTL — default INT1 source (overridden by start_rx for ISR mode)
  ESP_LOGVV(TAG, "fixup 8d: CUS_INT1_CTL = 0x%02X (TX_FIFO_NMTY)", CMT2300A_INT_SEL_TX_FIFO_NMTY);
  this->set_int1_source_(CMT2300A_INT_SEL_TX_FIFO_NMTY);

  // Firmware ends fixups by going to SLEEP
  // Caller transitions to needed mode afterward
  ESP_LOGVV(TAG, "fixups done, going to SLEEP");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_SLEEP);
}

void CMT2300A::set_int1_source_(uint8_t source) { this->write_reg(CMT2300A_CUS_INT1_CTL, source); }

// ============================================================================
// Mode Control
// ============================================================================

bool CMT2300A::wait_for_mode_(uint8_t expected_mode, uint32_t timeout_ms) {
  uint32_t start = millis();
  ESP_LOGVV(TAG, "wait_for_mode_: expecting 0x%02X, timeout %ums", expected_mode, timeout_ms);
  while (millis() - start < timeout_ms) {
    uint8_t raw = this->read_reg(CMT2300A_CUS_MODE_STA);
    uint8_t status = raw & CMT2300A_MASK_CHIP_MODE_STA;
    if (status == expected_mode) {
      ESP_LOGVV(TAG, "wait_for_mode_: reached 0x%02X in %ums (raw MODE_STA=0x%02X)", expected_mode,
                millis() - start, raw);
      return true;
    }
    delayMicroseconds(100);
  }
  uint8_t final_raw = this->read_reg(CMT2300A_CUS_MODE_STA);
  ESP_LOGW(TAG, "Timeout waiting for mode 0x%02X after %ums (current MODE_STA=0x%02X, mode=0x%02X)",
           expected_mode, timeout_ms, final_raw, final_raw & CMT2300A_MASK_CHIP_MODE_STA);
  return false;
}

void CMT2300A::clear_interrupts_() {
  // Clear all interrupt flags by writing clear bits
  // CUS_INT_CLR1: clear TX_DONE, SL_TMO, RX_TMO
  this->write_reg(CMT2300A_CUS_INT_CLR1,
                  CMT2300A_MASK_TX_DONE_CLR | CMT2300A_MASK_SL_TMO_CLR | CMT2300A_MASK_RX_TMO_CLR);
  // CUS_INT_CLR2: clear LBD, PREAM_OK, SYNC_OK, NODE_OK, CRC_OK, PKT_DONE
  this->write_reg(CMT2300A_CUS_INT_CLR2, CMT2300A_MASK_LBD_CLR | CMT2300A_MASK_PREAM_OK_CLR |
                                             CMT2300A_MASK_SYNC_OK_CLR | CMT2300A_MASK_NODE_OK_CLR |
                                             CMT2300A_MASK_CRC_OK_CLR | CMT2300A_MASK_PKT_DONE_CLR);
}

void CMT2300A::clear_fifo_() {
  // Per firmware at 0x13E16/0x13E2C: write to CUS_FIFO_CLR
  // Write both to clear entire FIFO
  this->write_reg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_TX);
  this->write_reg(CMT2300A_CUS_FIFO_CLR, CMT2300A_MASK_FIFO_CLR_RX);
}

bool CMT2300A::go_standby() {
  ESP_LOGVV(TAG, "go_standby: sending GO_STBY");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  bool ok = this->wait_for_mode_(CMT2300A_STA_STBY);
  if (!ok) {
    ESP_LOGW(TAG, "go_standby: FAILED to reach STBY");
  }
  return ok;
}

// ============================================================================
// ISR-driven RX (ESP32 only)
// ============================================================================

#ifdef USE_ESP32

void IRAM_ATTR CMT2300A::gpio1_isr_(CMT2300A *arg) {
  // ISR context — only send notification, no SPI here (bit-banged GPIO not ISR-safe)
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(arg->rx_task_handle_, &woken);
  portYIELD_FROM_ISR(woken);
}

void CMT2300A::receiver_task_(void *arg) {
  CMT2300A *radio = (CMT2300A *) arg;

  while (true) {
    // Block until GPIO1 ISR fires or 60s timeout (safety watchdog)
    uint32_t notification = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(60000));

    if (!radio->rx_active_)
      continue;  // not in RX mode, ignore stale interrupts

    if (notification == 0) {
      // 60s timeout with no packet — restart RX to recover from stuck state
      ESP_LOGD(TAG, "RX task watchdog — restarting RX");
      radio->clear_interrupts_();
      radio->clear_fifo_();
      radio->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
      continue;
    }

    // ISR fired — read interrupt flags via SPI (safe in task context)
    uint8_t flags = radio->read_reg(CMT2300A_CUS_INT_FLAG);

    if (flags & CMT2300A_MASK_PKT_OK_FLG) {
      uint8_t pkt_len = radio->read_reg(CMT2300A_CUS_PKT7);
      if (pkt_len > 0 && pkt_len <= CMT_MAX_PKT_SIZE) {
        RxPacket pkt;
        pkt.len = pkt_len;
        radio->read_fifo(pkt.data, pkt_len);
        ESP_LOGD(TAG, "RX task: %d bytes", pkt_len);

        // Non-blocking push to queue — drop if full
        if (xQueueSend(radio->rx_queue_, &pkt, 0) != pdTRUE) {
          ESP_LOGW(TAG, "RX queue full, packet dropped");
        }
      } else {
        ESP_LOGW(TAG, "RX task: bad length %d", pkt_len);
      }

      // Clear and re-enter RX for next packet
      radio->clear_interrupts_();
      radio->clear_fifo_();
      radio->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);

    } else if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
      ESP_LOGW(TAG, "RX task: CRC error");
      radio->clear_interrupts_();
      radio->clear_fifo_();
      radio->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);

    } else {
      // Spurious interrupt — clear and continue
      radio->clear_interrupts_();
    }
  }
}

bool CMT2300A::init_isr_() {
  if (this->pin_gpio1_ == nullptr) {
    ESP_LOGV(TAG, "init_isr_: pin_gpio1_ is null, cannot use ISR");
    return false;
  }

  ESP_LOGV(TAG, "init_isr_: creating RX queue (depth=3, item_size=%d)", (int) sizeof(RxPacket));
  // Create packet queue (depth 3)
  this->rx_queue_ = xQueueCreate(3, sizeof(RxPacket));
  if (this->rx_queue_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create RX queue");
    return false;
  }
  ESP_LOGVV(TAG, "init_isr_: RX queue created OK");

  // Create receiver task (3KB stack, priority 2 — above idle, below WiFi)
  ESP_LOGV(TAG, "init_isr_: creating receiver task (3KB stack, prio 2, core 1)");
  BaseType_t ret = xTaskCreatePinnedToCore(receiver_task_, "cmt_rx", 3 * 1024, this, 2, &this->rx_task_handle_, 1);
  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create RX task (ret=%d)", ret);
    vQueueDelete(this->rx_queue_);
    this->rx_queue_ = nullptr;
    return false;
  }
  ESP_LOGVV(TAG, "init_isr_: receiver task created OK");

  // Attach GPIO1 interrupt (RISING edge = INT1 asserted)
  ESP_LOGV(TAG, "init_isr_: attaching GPIO1 interrupt (RISING edge)");
  this->pin_gpio1_->attach_interrupt(gpio1_isr_, this, gpio::INTERRUPT_RISING_EDGE);

  this->isr_enabled_ = true;
  ESP_LOGD(TAG, "ISR-driven RX enabled on GPIO1");
  return true;
}

void CMT2300A::deinit_isr_() {
  if (!this->isr_enabled_)
    return;

  this->pin_gpio1_->detach_interrupt();

  if (this->rx_task_handle_ != nullptr) {
    vTaskDelete(this->rx_task_handle_);
    this->rx_task_handle_ = nullptr;
  }
  if (this->rx_queue_ != nullptr) {
    vQueueDelete(this->rx_queue_);
    this->rx_queue_ = nullptr;
  }

  this->isr_enabled_ = false;
}

#else  // !USE_ESP32

bool CMT2300A::init_isr_() { return false; }
void CMT2300A::deinit_isr_() {}

#endif  // USE_ESP32

// ============================================================================
// Initialization
// ============================================================================

bool CMT2300A::init(uint8_t channel) {
  ESP_LOGD(TAG, "Initializing CMT2300A, channel %d (%.3f MHz)", channel, CMT2300A_FREQ_MHZ[channel < 4 ? channel : 1]);

  // Setup GPIO pins
  ESP_LOGV(TAG, "init: setting up GPIO pins");
  ESP_LOGVV(TAG, "init: pin_sdio_=%p pin_sclk_=%p pin_csb_=%p pin_fcsb_=%p pin_gpio1_=%p",
            this->pin_sdio_, this->pin_sclk_, this->pin_csb_, this->pin_fcsb_, this->pin_gpio1_);
  if (this->pin_sdio_ == nullptr || this->pin_sclk_ == nullptr ||
      this->pin_csb_ == nullptr || this->pin_fcsb_ == nullptr) {
    ESP_LOGE(TAG, "init: one or more SPI pins are null!");
    return false;
  }

  this->pin_sdio_->setup();
  this->pin_sclk_->setup();
  this->pin_csb_->setup();
  this->pin_fcsb_->setup();
  ESP_LOGVV(TAG, "init: pin setup() done");

  this->pin_sdio_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_sclk_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_csb_->pin_mode(gpio::FLAG_OUTPUT);
  this->pin_fcsb_->pin_mode(gpio::FLAG_OUTPUT);
  ESP_LOGVV(TAG, "init: pin_mode(OUTPUT) done");

  // Set idle states
  this->pin_csb_->digital_write(true);
  this->pin_fcsb_->digital_write(true);
  this->pin_sclk_->digital_write(false);
  ESP_LOGVV(TAG, "init: idle states set (CSB=1, FCSB=1, SCLK=0)");

  // Allow SPI interface to sync (firmware sends 10 clock pulses; delay is equivalent)
  delay(1);

  if (this->pin_gpio1_ != nullptr) {
    ESP_LOGV(TAG, "init: GPIO1 pin provided, setting up as input");
    this->pin_gpio1_->setup();
    this->pin_gpio1_->pin_mode(gpio::FLAG_INPUT);
  } else {
    ESP_LOGV(TAG, "init: GPIO1 pin is null — ISR mode will not be available");
  }

  // Soft reset: firmware writes 0xFF to CUS_SOFTRST, then go_standby
  // See firmware function 0x13C7C
  ESP_LOGV(TAG, "init: soft reset (writing 0xFF to CUS_SOFTRST)");
  this->write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(20);
  ESP_LOGV(TAG, "init: sending GO_STBY after reset");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_STBY);
  delay(20);

  ESP_LOGV(TAG, "init: waiting for STBY mode...");
  if (!this->wait_for_mode_(CMT2300A_STA_STBY, 100)) {
    ESP_LOGE(TAG, "CMT2300A failed to enter standby mode after reset — chip not connected or wrong pins?");
    uint8_t mode_sta = this->read_reg(CMT2300A_CUS_MODE_STA);
    ESP_LOGE(TAG, "  CUS_MODE_STA=0x%02X (mode bits=0x%X)", mode_sta, mode_sta & CMT2300A_MASK_CHIP_MODE_STA);
    return false;
  }
  ESP_LOGV(TAG, "init: STBY mode reached after reset");

  // Write RX config as default idle config
  ESP_LOGV(TAG, "init: writing RX config (96 registers, ch=%d)", channel);
  this->write_config_(CMT2300A_RX_CONFIG, channel);
  ESP_LOGV(TAG, "init: RX config written, chip should be in SLEEP");

  // write_config_ ends in SLEEP, return to STBY
  ESP_LOGV(TAG, "init: going back to STBY after config");
  if (!this->go_standby()) {
    ESP_LOGE(TAG, "CMT2300A failed to return to standby after config");
    uint8_t mode_sta = this->read_reg(CMT2300A_CUS_MODE_STA);
    ESP_LOGE(TAG, "  CUS_MODE_STA=0x%02X (mode bits=0x%X)", mode_sta, mode_sta & CMT2300A_MASK_CHIP_MODE_STA);
    return false;
  }
  ESP_LOGV(TAG, "init: STBY mode reached after config");

  ESP_LOGVV(TAG, "init: clearing interrupts and FIFO");
  this->clear_interrupts_();
  this->clear_fifo_();

  // Verify chip: read back a known register
  uint8_t check = this->read_reg(CMT2300A_CUS_CMT1);
  ESP_LOGV(TAG, "init: chip verify — CUS_CMT1(reg 0x00) = 0x%02X", check);
  if (check == 0xFF || check == 0x00) {
    ESP_LOGE(TAG, "CMT2300A not responding (read 0x%02X from reg 0x00) — check SPI wiring!", check);
    return false;
  }

  // Read back a few more regs for diagnostics
  uint8_t reg_mode_sta = this->read_reg(CMT2300A_CUS_MODE_STA);
  uint8_t reg_int_en = this->read_reg(CMT2300A_CUS_INT_EN);
  uint8_t reg_io_sel = this->read_reg(CMT2300A_CUS_IO_SEL);
  uint8_t reg_fifo_ctl = this->read_reg(CMT2300A_CUS_FIFO_CTL);
  ESP_LOGV(TAG, "init: post-config regs: MODE_STA=0x%02X INT_EN=0x%02X IO_SEL=0x%02X FIFO_CTL=0x%02X",
           reg_mode_sta, reg_int_en, reg_io_sel, reg_fifo_ctl);

  // Try to initialize ISR-driven RX (falls back to polling if GPIO1 not available)
  ESP_LOGV(TAG, "init: attempting ISR setup...");
  if (!this->init_isr_()) {
    ESP_LOGD(TAG, "GPIO1 not available — using polling RX");
  } else {
    ESP_LOGV(TAG, "init: ISR-driven RX enabled successfully");
  }

  ESP_LOGD(TAG, "CMT2300A initialized OK (reg0=0x%02X, isr=%s)", check, this->isr_enabled_ ? "yes" : "no");
  return true;
}

// ============================================================================
// TX / RX Mode Switching
// ============================================================================

bool CMT2300A::switch_tx(uint8_t channel) {
  ESP_LOGV(TAG, "switch_tx: ch=%d", channel);

  // Soft reset before config switch — firmware does this in pre_config() (0x13FEA)
  ESP_LOGVV(TAG, "switch_tx: soft reset before config");
  this->write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(10);

  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_tx: go_standby failed (pre-config)");
    return false;
  }

  this->write_config_(CMT2300A_TX_CONFIG, channel);

  // write_config_ ends in SLEEP (per firmware). Return to STBY for FIFO access.
  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_tx: go_standby failed (post-config)");
    return false;
  }

  this->clear_interrupts_();
  this->clear_fifo_();
  ESP_LOGV(TAG, "switch_tx: ready");
  return true;
}

bool CMT2300A::switch_rx(uint8_t channel) {
  ESP_LOGV(TAG, "switch_rx: ch=%d", channel);

  // Soft reset before config switch — firmware does this in pre_config() (0x13FEA)
  ESP_LOGVV(TAG, "switch_rx: soft reset before config");
  this->write_reg(CMT2300A_CUS_SOFTRST, 0xFF);
  delay(10);

  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_rx: go_standby failed (pre-config)");
    return false;
  }

  this->write_config_(CMT2300A_RX_CONFIG, channel);

  // write_config_ ends in SLEEP (per firmware). Return to STBY for FIFO access.
  if (!this->go_standby()) {
    ESP_LOGW(TAG, "switch_rx: go_standby failed (post-config)");
    return false;
  }

  this->clear_interrupts_();
  this->clear_fifo_();
  ESP_LOGV(TAG, "switch_rx: ready");
  return true;
}

// ============================================================================
// Packet TX (blocking — OK, TX is fast: ~100ms max for W-MBus frame)
// ============================================================================

bool CMT2300A::send_packet(const uint8_t *data, uint16_t len, uint8_t channel) {
  ESP_LOGD(TAG, "TX %d bytes on ch%d", len, channel);
  ESP_LOGV(TAG, "send_packet: first bytes: %02X %02X %02X %02X ...",
           len > 0 ? data[0] : 0, len > 1 ? data[1] : 0, len > 2 ? data[2] : 0, len > 3 ? data[3] : 0);

  if (!this->switch_tx(channel)) {
    ESP_LOGW(TAG, "send_packet: switch_tx failed");
    return false;
  }

  // Write data to FIFO
  ESP_LOGV(TAG, "send_packet: writing %d bytes to FIFO", len);
  this->write_fifo(data, len);

  // Set packet length (CUS_PKT7 stores payload length)
  this->write_reg(CMT2300A_CUS_PKT7, len);

  // Go TX
  this->clear_interrupts_();
  ESP_LOGV(TAG, "send_packet: sending GO_TX");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_TX);

  // Wait for TX done (poll status, ~100ms max for typical W-MBus frame)
  uint32_t start = millis();
  while (millis() - start < 200) {
    // TX_DONE flag is in CUS_INT_CLR1 (0x6A) bit 3 (read-only)
    uint8_t clr1 = this->read_reg(CMT2300A_CUS_INT_CLR1);
    if (clr1 & CMT2300A_MASK_TX_DONE_FLG) {
      ESP_LOGD(TAG, "TX complete in %ums", millis() - start);
      this->go_standby();
      return true;
    }
    // Also check mode status — if back to standby, TX is done
    uint8_t mode = this->read_reg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
    if (mode == CMT2300A_STA_STBY) {
      ESP_LOGD(TAG, "TX complete in %ums (mode=STBY)", millis() - start);
      return true;
    }
    delayMicroseconds(500);
  }

  uint8_t final_mode = this->read_reg(CMT2300A_CUS_MODE_STA) & CMT2300A_MASK_CHIP_MODE_STA;
  uint8_t final_flags = this->read_reg(CMT2300A_CUS_INT_FLAG);
  ESP_LOGW(TAG, "TX timeout after 200ms (mode=0x%02X, flags=0x%02X)", final_mode, final_flags);
  this->go_standby();
  return false;
}

// ============================================================================
// Blocking RX (used by sniffer mode only)
// ============================================================================

uint16_t CMT2300A::receive_packet(uint8_t *buf, uint16_t max_len, uint32_t timeout_ms, uint8_t channel) {
  ESP_LOGVV(TAG, "receive_packet: ch=%d timeout=%ums", channel, timeout_ms);
  if (!this->switch_rx(channel)) {
    ESP_LOGW(TAG, "receive_packet: switch_rx failed");
    return 0;
  }

  // Enter RX mode
  this->clear_interrupts_();
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
  ESP_LOGVV(TAG, "receive_packet: entered RX mode, polling...");

  // Poll for packet received
  uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    uint8_t flags = this->read_reg(CMT2300A_CUS_INT_FLAG);
    if (flags & CMT2300A_MASK_PKT_OK_FLG) {
      // Read packet length from FIFO status
      uint8_t pkt_len = this->read_reg(CMT2300A_CUS_PKT7);  // RX payload length register
      if (pkt_len == 0 || pkt_len > max_len) {
        ESP_LOGW(TAG, "RX bad length: %d (max=%d)", pkt_len, max_len);
        this->go_standby();
        return 0;
      }

      this->read_fifo(buf, pkt_len);
      ESP_LOGD(TAG, "RX %d bytes in %ums", pkt_len, millis() - start);
      ESP_LOGV(TAG, "RX first bytes: %02X %02X %02X %02X ...",
               pkt_len > 0 ? buf[0] : 0, pkt_len > 1 ? buf[1] : 0,
               pkt_len > 2 ? buf[2] : 0, pkt_len > 3 ? buf[3] : 0);
      this->go_standby();
      return pkt_len;
    }

    // Check for packet error
    if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
      ESP_LOGW(TAG, "RX packet error (flags=0x%02X) at %ums", flags, millis() - start);
      this->clear_interrupts_();
      this->clear_fifo_();
      // Re-enter RX
      this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
    }

    delay(1);
  }

  ESP_LOGVV(TAG, "receive_packet: timeout, no packet in %ums", timeout_ms);
  this->go_standby();
  return 0;
}

// ============================================================================
// Non-blocking RX API
// ============================================================================

bool CMT2300A::start_rx(uint8_t channel) {
  ESP_LOGV(TAG, "start_rx: ch=%d isr=%s", channel, this->isr_enabled_ ? "yes" : "no");
  if (!this->switch_rx(channel)) {
    ESP_LOGW(TAG, "start_rx: switch_rx failed");
    return false;
  }

  // Configure INT1 for PKT_DONE (fires on both OK and error packets)
  if (this->isr_enabled_) {
    ESP_LOGVV(TAG, "start_rx: setting INT1 source to PKT_DONE for ISR");
    this->set_int1_source_(CMT2300A_INT_SEL_PKT_DONE);
  }

  this->clear_interrupts_();
  ESP_LOGVV(TAG, "start_rx: sending GO_RX");
  this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);
  this->rx_active_ = true;

#ifdef USE_ESP32
  // Drain any stale packets from queue
  if (this->isr_enabled_ && this->rx_queue_ != nullptr) {
    RxPacket discard;
    int drained = 0;
    while (xQueueReceive(this->rx_queue_, &discard, 0) == pdTRUE) {
      drained++;
    }
    if (drained > 0) {
      ESP_LOGV(TAG, "start_rx: drained %d stale packets from queue", drained);
    }
  }
#endif

  ESP_LOGV(TAG, "start_rx: now listening on ch%d", channel);
  return true;
}

int16_t CMT2300A::check_rx(uint8_t *buf, uint16_t max_len) {
  if (this->isr_enabled_) {
    return this->check_rx_isr_(buf, max_len);
  }
  return this->check_rx_polling_(buf, max_len);
}

void CMT2300A::stop_rx() {
  ESP_LOGV(TAG, "stop_rx: stopping RX, isr=%s", this->isr_enabled_ ? "yes" : "no");
  this->rx_active_ = false;

  // Restore INT1 source to default (TX_FIFO_NMTY)
  if (this->isr_enabled_) {
    this->set_int1_source_(CMT2300A_INT_SEL_TX_FIFO_NMTY);
  }

  this->go_standby();
}

// Polling fallback: read interrupt flags via SPI each call (~50μs)
int16_t CMT2300A::check_rx_polling_(uint8_t *buf, uint16_t max_len) {
  uint8_t flags = this->read_reg(CMT2300A_CUS_INT_FLAG);

  if (flags & CMT2300A_MASK_PKT_OK_FLG) {
    uint8_t pkt_len = this->read_reg(CMT2300A_CUS_PKT7);
    ESP_LOGV(TAG, "check_rx_poll: PKT_OK, len=%d, flags=0x%02X", pkt_len, flags);
    if (pkt_len == 0 || pkt_len > max_len) {
      ESP_LOGW(TAG, "RX bad length: %d (max=%d)", pkt_len, max_len);
      return -1;
    }
    this->read_fifo(buf, pkt_len);
    ESP_LOGD(TAG, "RX %d bytes (polling)", pkt_len);
    ESP_LOGV(TAG, "RX first bytes: %02X %02X %02X %02X ...",
             pkt_len > 0 ? buf[0] : 0, pkt_len > 1 ? buf[1] : 0,
             pkt_len > 2 ? buf[2] : 0, pkt_len > 3 ? buf[3] : 0);
    return pkt_len;
  }

  if (flags & CMT2300A_MASK_PKT_ERR_FLG) {
    ESP_LOGW(TAG, "RX packet error (polling, flags=0x%02X)", flags);
    this->clear_interrupts_();
    this->clear_fifo_();
    this->write_reg(CMT2300A_CUS_MODE_CTL, CMT2300A_GO_RX);  // re-enter RX
  }

  return 0;  // nothing yet
}

// ISR-driven: instant check of FreeRTOS queue (no SPI)
int16_t CMT2300A::check_rx_isr_(uint8_t *buf, uint16_t max_len) {
#ifdef USE_ESP32
  if (this->rx_queue_ == nullptr)
    return 0;

  RxPacket pkt;
  if (xQueueReceive(this->rx_queue_, &pkt, 0) != pdTRUE)
    return 0;  // nothing yet — instant return

  if (pkt.len > max_len) {
    ESP_LOGW(TAG, "RX packet too large for buffer: %d > %d", pkt.len, max_len);
    return -1;
  }

  memcpy(buf, pkt.data, pkt.len);
  return pkt.len;
#else
  return this->check_rx_polling_(buf, max_len);
#endif
}

}  // namespace esphome::nartis_wmbus
