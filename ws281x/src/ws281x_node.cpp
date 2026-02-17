/*
 * ws281x LED strip ROS driver for Orange Pi 5 (SPI based)
 * Ported to match Python 2.4MHz Oversampling method
 */

#include <ros/ros.h>
#include <led_msgs/SetLEDs.h>
#include <led_msgs/LEDStateArray.h>
#include <ws281x/SetGamma.h>
#include <ros/console.h>

#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <cstring>
#include <vector>
#include <string>
#include <unordered_map>
#include <algorithm>

// ==========================================
// SPI SETTINGS (MATCHING PYTHON DRIVER)
// ==========================================
// Частота 2.4 МГц позволяет кодировать 1 бит WS2812 тремя битами SPI
// 0 -> 100 (0x4)
// 1 -> 110 (0x6)
constexpr uint32_t SPI_SPEED_HZ = 2400000;
constexpr uint32_t RESET_TIME_US = 300; // 300us reset (Safe for WS2812B)

enum StripType {
    STRIP_RGB = 0,
    STRIP_RBG,
    STRIP_GRB,
    STRIP_GBR,
    STRIP_BRG,
    STRIP_BGR,
    STRIP_RGBW, // SK6812 variants
    STRIP_SK6812_GRBW
};

struct PixelColor {
    uint8_t w;
    uint8_t r;
    uint8_t g;
    uint8_t b;
};

class WS2812SPI {
public:
    WS2812SPI() : fd_(-1), num_leds_(0), brightness_(255) {
        // Инициализация гамма-таблицы (линейная по умолчанию)
        for (int i = 0; i < 256; i++) gamma_table_[i] = i;
        
        // Генерация таблицы перекодировки (как в Python скрипте)
        // 1 байт цвета -> 3 байта SPI
        generateLookupTable();
    }

    ~WS2812SPI() {
        if (fd_ >= 0) {
            // Выключаем диоды при выходе
            std::fill(pixels_.begin(), pixels_.end(), PixelColor{0,0,0,0});
            render();
            close(fd_);
        }
    }

    bool init(const std::string& device, int num_leds, StripType type) {
        num_leds_ = num_leds;
        type_ = type;
        pixels_.resize(num_leds, {0, 0, 0, 0});

        is_rgbw_ = (type_ >= STRIP_RGBW);
        // 1 пиксель = (3 или 4 цвета) * 3 байта SPI на каждый цвет
        int spi_bytes_per_pixel = (is_rgbw_ ? 4 : 3) * 3;
        
        // Ресайз буфера
        spi_buffer_.resize(num_leds * spi_bytes_per_pixel);

        fd_ = open(device.c_str(), O_RDWR);
        if (fd_ < 0) {
            ROS_ERROR("Failed to open SPI device: %s", device.c_str());
            return false;
        }

        uint8_t mode = 0;
        uint8_t bits = 8;
        uint32_t speed = SPI_SPEED_HZ;

        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) return false;
        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;
        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) return false;

        return true;
    }

    void setPixel(int index, uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {
        if (index >= 0 && index < num_leds_) {
            pixels_[index] = {w, r, g, b};
        }
    }

    PixelColor getPixel(int index) {
        if (index >= 0 && index < num_leds_) return pixels_[index];
        return {0,0,0,0};
    }

    void setBrightness(uint8_t b) {
        brightness_ = b;
    }

    void setGamma(const std::vector<uint8_t>& gamma) {
        if (gamma.size() == 256) {
            for(int i=0; i<256; i++) gamma_table_[i] = gamma[i];
        }
    }

    bool render() {
        if (fd_ < 0) return false;

        size_t buf_idx = 0;
        
        for (const auto& p : pixels_) {
            // Применяем яркость и гамму
            uint8_t r = gamma_table_[(p.r * brightness_) >> 8];
            uint8_t g = gamma_table_[(p.g * brightness_) >> 8];
            uint8_t b = gamma_table_[(p.b * brightness_) >> 8];
            uint8_t w = gamma_table_[(p.w * brightness_) >> 8];

            // Определяем порядок цветов и заполняем буфер используя lookup table
            // На каждый компонент цвета уходит 3 байта в буфере SPI
            
            // Helper lambda to append 3 bytes
            auto append_byte = [&](uint8_t val) {
                spi_buffer_[buf_idx++] = lookup_table_[val][0];
                spi_buffer_[buf_idx++] = lookup_table_[val][1];
                spi_buffer_[buf_idx++] = lookup_table_[val][2];
            };

            switch (type_) {
                case STRIP_RGB: append_byte(r); append_byte(g); append_byte(b); break;
                case STRIP_RBG: append_byte(r); append_byte(b); append_byte(g); break;
                case STRIP_GRB: append_byte(g); append_byte(r); append_byte(b); break; // Стандартный WS2812
                case STRIP_GBR: append_byte(g); append_byte(b); append_byte(r); break;
                case STRIP_BRG: append_byte(b); append_byte(r); append_byte(g); break;
                case STRIP_BGR: append_byte(b); append_byte(g); append_byte(r); break;
                case STRIP_RGBW: append_byte(r); append_byte(g); append_byte(b); append_byte(w); break;
                case STRIP_SK6812_GRBW: append_byte(g); append_byte(r); append_byte(b); append_byte(w); break;
                default: append_byte(g); append_byte(r); append_byte(b); break;
            }
        }

        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)spi_buffer_.data(),
            .rx_buf = 0,
            .len = (uint32_t)spi_buffer_.size(),
            .speed_hz = SPI_SPEED_HZ,
            .delay_usecs = 0,
            .bits_per_word = 8,
        };

        int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
        
        // Важно: пауза для Reset сигнала (Latch), как в Python скрипте
        usleep(RESET_TIME_US); 
        
        return ret >= 0;
    }

    int getCount() const { return num_leds_; }

private:
    void generateLookupTable() {
        // Создаем таблицу, где каждый байт (0-255) превращается в 3 байта SPI
        for (int i = 0; i < 256; i++) {
            uint32_t spi_bits = 0;
            // Проходим по битам от старшего к младшему
            for (int bit = 7; bit >= 0; bit--) {
                // Если бит 1 -> паттерн 110, если 0 -> паттерн 100
                uint8_t pattern = ((i >> bit) & 1) ? 0b110 : 0b100;
                spi_bits = (spi_bits << 3) | pattern;
            }
            // Разбиваем 24 бита на 3 байта
            lookup_table_[i][0] = (spi_bits >> 16) & 0xFF;
            lookup_table_[i][1] = (spi_bits >> 8) & 0xFF;
            lookup_table_[i][2] = spi_bits & 0xFF;
        }
    }

    int fd_;
    int num_leds_;
    uint8_t brightness_;
    StripType type_;
    bool is_rgbw_;
    std::vector<PixelColor> pixels_;
    std::vector<uint8_t> spi_buffer_;
    uint8_t gamma_table_[256];
    
    // Таблица: индекс [0-255], значение [3 байта SPI]
    uint8_t lookup_table_[256][3]; 
};

// --- SPI DRIVER IMPLEMENTATION END ---


// Global instance
WS2812SPI led_driver;

ros::Publisher led_state_pub;
led_msgs::LEDStateArray strip_state;

// Map string types to our enum
std::unordered_map<std::string, StripType> strip_types_map = {
    {"SK6812_STRIP_RGBW", STRIP_RGBW},
    {"SK6812_STRIP_GRBW", STRIP_SK6812_GRBW},
    {"WS2811_STRIP_RGB", STRIP_RGB},
    {"WS2811_STRIP_RBG", STRIP_RBG},
    {"WS2811_STRIP_GRB", STRIP_GRB},
    {"WS2811_STRIP_GBR", STRIP_GBR},
    {"WS2811_STRIP_BRG", STRIP_BRG},
    {"WS2811_STRIP_BGR", STRIP_BGR},
    {"WS2812_STRIP", STRIP_GRB}, // Standard WS2812 is usually GRB
    {"SK6812_STRIP", STRIP_GRB},
};

void publishLedState()
{
    strip_state.leds.resize(led_driver.getCount());
    for(int i = 0; i < led_driver.getCount(); ++i) {
        PixelColor p = led_driver.getPixel(i);
        strip_state.leds[i].index = i;
        strip_state.leds[i].r = p.r;
        strip_state.leds[i].g = p.g;
        strip_state.leds[i].b = p.b;
    }
    led_state_pub.publish(strip_state);
}

bool setGamma(ws281x::SetGamma::Request& req, ws281x::SetGamma::Response& resp)
{
    std::vector<uint8_t> g(256);
    for(int i = 0; i < 256; ++i) {
        g[i] = req.gamma[i];
    }
    led_driver.setGamma(g);
    resp.success = 1;
    return true;
}

bool setLeds(led_msgs::SetLEDs::Request& req, led_msgs::SetLEDs::Response& resp)
{
    for(auto const& led : req.leds) {
        if (led.index < 0 || led.index >= led_driver.getCount()) {
            ROS_ERROR("[ws281x] LED index out of bounds: %d", led.index);
            resp.message = "LED index out of bounds: " + std::to_string(led.index);
            return true;
        }
    }

    for(auto const& led : req.leds) {
        led_driver.setPixel(led.index, led.r, led.g, led.b, 0);
    }

    if (!led_driver.render()) {
        resp.message = "SPI write failed";
        ROS_ERROR_THROTTLE(1, "[ws281x] Could not set LED colors: %s", resp.message.c_str());
        resp.success = false;
    } else {
        resp.success = true;
        resp.message = "";
    }
    publishLedState();
    return true;
}

void cleanup(int signal)
{
    (void) signal;
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ws281x");
    ros::NodeHandle nh, nh_priv("~");

    int param_led_count;
    int param_brightness;
    std::string param_strip_type_str;
    std::string param_spi_device;

    // Unused params kept for compatibility
    int param_freq, param_dma, param_pin;
    bool param_invert;

    nh_priv.param("led_count", param_led_count, 30);
    nh_priv.param("brightness", param_brightness, 255);
    nh_priv.param("strip_type", param_strip_type_str, std::string("WS2811_STRIP_GRB"));
    nh_priv.param("spi_device", param_spi_device, std::string("/dev/spidev4.1"));

    nh_priv.param("target_frequency", param_freq, 800000);
    nh_priv.param("gpio_pin", param_pin, 21);
    nh_priv.param("dma", param_dma, 10);
    nh_priv.param("invert", param_invert, false);

    if (nh_priv.hasParam("gpio_pin") || nh_priv.hasParam("dma")) {
        ROS_WARN("[ws281x] Legacy params ignored. Using SPI logic (2.4MHz oversampling) on %s", param_spi_device.c_str());
    }

    StripType strip_type = STRIP_GRB;
    auto it = strip_types_map.find(param_strip_type_str);
    if (it != strip_types_map.end()) {
        strip_type = it->second;
    } else {
        ROS_WARN("[ws281x] Unknown strip type: %s, defaulting to GRB", param_strip_type_str.c_str());
    }

    ROS_INFO("[ws281x] Initializing SPI (2.4MHz): %s, LEDs: %d, Type: %s",
             param_spi_device.c_str(), param_led_count, param_strip_type_str.c_str());

    if (!led_driver.init(param_spi_device, param_led_count, strip_type)) {
        ROS_FATAL("[ws281x] Failed to initialize SPI driver. Check permissions and path.");
        return 1;
    }

    led_driver.setBrightness((uint8_t)param_brightness);

    signal(SIGINT, cleanup);
    signal(SIGTERM, cleanup);

    auto srv_gamma = nh_priv.advertiseService("set_gamma", setGamma);
    auto srv_leds = nh_priv.advertiseService("set_leds", setLeds);
    led_state_pub = nh_priv.advertise<led_msgs::LEDStateArray>("state", 1, true);

    publishLedState();

    ros::spin();

    return 0;
}