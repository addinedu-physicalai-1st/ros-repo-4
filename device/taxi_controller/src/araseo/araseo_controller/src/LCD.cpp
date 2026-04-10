#include "LCD.hpp"
#include <iostream>
#include <cstring>

LCD::LCD(int chip_id) : w(240), h(320), chip_id(chip_id), h_gpio(-1), spi_fd(-1) {}

LCD::~LCD() {
    close_lcd();
}

bool LCD::init() {
    // 1. lgpio 초기화
    h_gpio = lgGpiochipOpen(chip_id);
    if (h_gpio < 0) {
        std::cerr << "[LCD] lgGpiochipOpen Error: " << h_gpio << std::endl;
        return false;
    }

    // 핀 설정
    lgGpioClaimOutput(h_gpio, 0, RST_PIN, 1);
    lgGpioClaimOutput(h_gpio, 0, DC_PIN, 1);
    lgGpioClaimOutput(h_gpio, 0, BL_PIN, 1);

    // 백라이트 PWM 설정 (1kHz)
    lgTxPwm(h_gpio, BL_PIN, 1000, 100, 0, 0);

    // 2. SPI 초기화
    spi_fd = open("/dev/spidev0.0", O_RDWR);
    if (spi_fd < 0) {
        std::cerr << "[LCD] SPI open error: /dev/spidev0.0" << std::endl;
        return false;
    }

    uint32_t speed = 80000000; // 80MHz
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    ioctl(spi_fd, SPI_IOC_WR_MODE, &mode);
    ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
    ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);

    // 3. LCD 하드웨어 초기화 (ILI9341)
    reset();
    write_cmd(0x11); // Sleep out
    usleep(120000);

    write_cmd(0xCF); write_data(0x00); write_data(0xC1); write_data(0X30);
    write_cmd(0xED); write_data(0x64); write_data(0x03); write_data(0X12); write_data(0X81);
    write_cmd(0xE8); write_data(0x85); write_data(0x00); write_data(0x79);
    write_cmd(0xCB); write_data(0x39); write_data(0x2C); write_data(0x00); write_data(0x34); write_data(0x02);
    write_cmd(0xF7); write_data(0x20);
    write_cmd(0xEA); write_data(0x00); write_data(0x00);

    write_cmd(0xC0); write_data(0x1D); // Power control
    write_cmd(0xC1); write_data(0x12); // Power control
    write_cmd(0xC5); write_data(0x33); write_data(0x3F); // VCM control
    write_cmd(0xC7); write_data(0x92); // VCM control

    write_cmd(0x3A); write_data(0x55); // Pixel Format (RGB565)
    write_cmd(0x36); write_data(0x08); // Memory Access Control (Standard orientation)

    write_cmd(0xB1); write_data(0x00); write_data(0x12);
    write_cmd(0xB6); write_data(0x0A); write_data(0xA2); // Display Function Control

    write_cmd(0x44); write_data(0x02);
    write_cmd(0xF2); write_data(0x00); // 3Gamma Function Disable
    write_cmd(0x26); write_data(0x01); // Gamma curve selected

    // Gamma Corection
    write_cmd(0xE0);
    uint8_t gammaP[] = {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x32, 0x44, 0x42, 0x06, 0x0E, 0x12, 0x14, 0x17};
    write_data_buffer(gammaP, sizeof(gammaP));

    write_cmd(0xE1);
    uint8_t gammaN[] = {0xD0, 0x00, 0x02, 0x07, 0x0A, 0x28, 0x31, 0x54, 0x47, 0x0E, 0x1F, 0x1B, 0x1A, 0x1F};
    write_data_buffer(gammaN, sizeof(gammaN));

    write_cmd(0x29); // Display on
    clear(0x0000);   // Black screen

    std::cout << "[LCD] C++ SPI Driver Initialized Successfully." << std::endl;
    return true;
}

void LCD::reset() {
    lgGpioWrite(h_gpio, RST_PIN, 1);
    usleep(10000);
    lgGpioWrite(h_gpio, RST_PIN, 0);
    usleep(10000);
    lgGpioWrite(h_gpio, RST_PIN, 1);
    usleep(10000);
}

void LCD::write_cmd(uint8_t cmd) {
    lgGpioWrite(h_gpio, DC_PIN, 0);
    write(spi_fd, &cmd, 1);
}

void LCD::write_data(uint8_t data) {
    lgGpioWrite(h_gpio, DC_PIN, 1);
    write(spi_fd, &data, 1);
}

void LCD::write_data_buffer(const uint8_t* buf, size_t len) {
    lgGpioWrite(h_gpio, DC_PIN, 1);
    
    // Large buffers need to be split if the SPI driver has limits, but usually spidev handles chunking.
    // However, spidev2_write is better for large buffers.
    write(spi_fd, buf, len);
}

void LCD::set_windows(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    write_cmd(0x2A);
    write_data(x0 >> 8); write_data(x0 & 0xFF);
    write_data((x1 - 1) >> 8); write_data((x1 - 1) & 0xFF);

    write_cmd(0x2B);
    write_data(y0 >> 8); write_data(y0 & 0xFF);
    write_data((y1 - 1) >> 8); write_data((y1 - 1) & 0xFF);

    write_cmd(0x2C);
}

void LCD::show(const cv::Mat& img) {
    // 1. 영상 가로세로 회전 (Python 로직은 transpose(ROTATE_270)와 FLIP_LEFT_RIGHT를 사용)
    cv::Mat rotated;
    cv::rotate(img, rotated, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::flip(rotated, rotated, 1); // FLIP_LEFT_RIGHT

    // 2. RGB565 변환
    cv::Mat img565;
    cv::cvtColor(rotated, img565, cv::COLOR_BGR2BGR565);

    // 3. 윈도우 설정 및 데이터 전송
    set_windows(0, 0, w, h);
    
    // SPI 전송 (엔디안 주의: BGR565는 기본적으로 uint16_t 리틀엔디안일 가능성이 큼)
    // 하지만 ST7789/ILI9341은 빅엔디안(High byte first)을 기대함.
    // OpenCV BGR565: 11111(B), 111111(G), 11111(R) ? 아님 5-6-5 순서임.
    // 수동 스왑 혹은 비트 연산이 필요할 수 있음.
    
    uint16_t* p = (uint16_t*)img565.data;
    size_t pixel_count = w * h;
    std::vector<uint8_t> buffer(pixel_count * 2);

    for (size_t i = 0; i < pixel_count; ++i) {
        uint16_t val = p[i];
        buffer[i * 2] = (val >> 8); // High byte
        buffer[i * 2 + 1] = (val & 0xFF); // Low byte
    }

    write_data_buffer(buffer.data(), buffer.size());
}

void LCD::clear(uint16_t color) {
    set_windows(0, 0, w, h);
    size_t pixel_count = w * h;
    std::vector<uint8_t> buffer(pixel_count * 2);
    uint8_t hi = color >> 8;
    uint8_t lo = color & 0xFF;
    for (size_t i = 0; i < pixel_count; ++i) {
        buffer[i * 2] = hi;
        buffer[i * 2 + 1] = lo;
    }
    write_data_buffer(buffer.data(), buffer.size());
}

void LCD::set_backlight(int value) {
    if (value < 0) value = 0;
    if (value > 100) value = 100;
    lgTxPwm(h_gpio, BL_PIN, 1000, value, 0, 0);
}

void LCD::close_lcd() {
    if (spi_fd >= 0) close(spi_fd);
    if (h_gpio >= 0) lgGpiochipClose(h_gpio);
    spi_fd = -1;
    h_gpio = -1;
}
