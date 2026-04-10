#ifndef LCD_HPP
#define LCD_HPP

#include <opencv2/opencv.hpp>
#include <lgpio.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>
#include <stdint.h>

class LCD {
private:
    int w, h;
    int h_gpio;
    int spi_fd;
    int chip_id;

    // Pin Assignments (BCM)
    const int RST_PIN = 27;
    const int DC_PIN = 25;
    const int BL_PIN = 18;

    void write_cmd(uint8_t cmd);
    void write_data(uint8_t data);
    void write_data_buffer(const uint8_t* buf, size_t len);
    void reset();
    void set_windows(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end);

public:
    LCD(int chip_id = 4);
    ~LCD();

    bool init();
    void show(const cv::Mat& img);
    void clear(uint16_t color = 0x0000);
    void set_backlight(int value);
    void close_lcd();
};

#endif // LCD_HPP
