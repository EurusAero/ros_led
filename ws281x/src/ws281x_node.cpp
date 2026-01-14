/*
 * ws281x LED strip ROS driver for Orange Pi 5 (SPI based)
 * Based on original code by Copter Express Technologies
 * Ported to Linux SPI
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


// SPI Speed for WS2812B emulation (approx 8MHz allows 1 byte per bit emulation)
// 0 bit: 0b11000000 (High ~0.25us) - actually at 8MHz 1 bit is 125ns.
// We need T0H ~350ns, T0L ~800ns. T1H ~700ns, T1L ~600ns.
// At 8MHz:
// 0 code: 11100000 (3 bits high = 375ns, 5 bits low = 625ns) -> Total 1000ns
// 1 code: 11111100 (6 bits high = 750ns, 2 bits low = 250ns) -> Total 1000ns
// This is within tolerance for WS2812B.

constexpr uint32_t SPI_SPEED_HZ = 8000000;
constexpr uint8_t SPI_BYTE_0 = 0b11100000;
constexpr uint8_t SPI_BYTE_1 = 0b11111100;

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
        // Initialize gamma table to linear by default
        for (int i = 0; i < 256; i++) gamma_table_[i] = i;
    }

    ~WS2812SPI() {
        if (fd_ >= 0) {
            // Turn off leds on exit
            std::fill(pixels_.begin(), pixels_.end(), PixelColor{0,0,0,0});
            render();
            close(fd_);
        }
    }

    bool init(const std::string& device, int num_leds, StripType type) {
        num_leds_ = num_leds;
        type_ = type;
        pixels_.resize(num_leds, {0, 0, 0, 0});

        // Determine bytes per pixel based on type
        is_rgbw_ = (type_ >= STRIP_RGBW);
        int bytes_per_pixel = is_rgbw_ ? 4 : 3;

        // Resize SPI buffer: (num_leds * bytes_per_pixel * 8 bits_per_byte) + reset_padding
        size_t spi_len = num_leds * bytes_per_pixel * 8;
        spi_buffer_.resize(spi_len + 100, 0); // +100 bytes for reset signal (low)

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

        int buf_idx = 0;
        for (const auto& p : pixels_) {
            // Apply brightness and gamma
            uint8_t r = gamma_table_[(p.r * brightness_) >> 8];
            uint8_t g = gamma_table_[(p.g * brightness_) >> 8];
            uint8_t b = gamma_table_[(p.b * brightness_) >> 8];
            uint8_t w = gamma_table_[(p.w * brightness_) >> 8];

            uint32_t color_ordered = 0;

            // Map logical colors to strip order
            switch (type_) {
                case STRIP_RGB: color_ordered = (r << 16) | (g << 8) | b; break;
                case STRIP_RBG: color_ordered = (r << 16) | (b << 8) | g; break;
                case STRIP_GRB: color_ordered = (g << 16) | (r << 8) | b; break;
                case STRIP_GBR: color_ordered = (g << 16) | (b << 8) | r; break;
                case STRIP_BRG: color_ordered = (b << 16) | (r << 8) | g; break;
                case STRIP_BGR: color_ordered = (b << 16) | (g << 8) | r; break;
                case STRIP_RGBW: color_ordered = (r << 24) | (g << 16) | (b << 8) | w; break;
                case STRIP_SK6812_GRBW: color_ordered = (g << 24) | (r << 16) | (b << 8) | w; break;
                default: color_ordered = (g << 16) | (r << 8) | b; break; // Default GRB
            }

            int bits_to_send = is_rgbw_ ? 32 : 24;

            // Convert bits to SPI bytes (MSB first)
            for (int i = bits_to_send - 1; i >= 0; i--) {
                if ((color_ordered >> i) & 1) {
                    spi_buffer_[buf_idx++] = SPI_BYTE_1;
                } else {
                    spi_buffer_[buf_idx++] = SPI_BYTE_0;
                }
            }
        }

        // Reset signal is handled by the zeros at the end of vector (resize fills with 0)

        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)spi_buffer_.data(),
            .rx_buf = 0,
            .len = (uint32_t)spi_buffer_.size(),
            .speed_hz = SPI_SPEED_HZ,
            .delay_usecs = 0,
            .bits_per_word = 8,
        };

        return ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 0;
    }

    int getCount() const { return num_leds_; }

private:
    int fd_;
    int num_leds_;
    uint8_t brightness_;
    StripType type_;
    bool is_rgbw_;
    std::vector<PixelColor> pixels_;
    std::vector<uint8_t> spi_buffer_;
    uint8_t gamma_table_[256];
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
        // led_msgs/LEDState doesn't usually have W, but we store it internally
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
    // check validness
    for(auto const& led : req.leds) {
        if (led.index < 0 || led.index >= led_driver.getCount()) {
            ROS_ERROR("[ws281x] LED index out of bounds: %d", led.index);
            resp.message = "LED index out of bounds: " + std::to_string(led.index);
            return true;
        }
    }

    for(auto const& led : req.leds) {
        // Note: led_msgs/LEDState usually doesn't have 'w', assuming RGB for now
        // If you need W support, you might need to extend the message or use a custom one.
        // Here we just set RGB.
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
    // Destructor of led_driver will handle cleanup (turn off leds)
    ros::shutdown();
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ws281x");
    ros::NodeHandle nh, nh_priv("~");

    // Parameters
    int param_led_count;
    int param_brightness;
    std::string param_strip_type_str;
    std::string param_spi_device;

    // Unused params kept for compatibility with launch files, but warned
    int param_freq, param_dma, param_pin;
    bool param_invert;

    nh_priv.param("led_count", param_led_count, 30);
    nh_priv.param("brightness", param_brightness, 255);
    nh_priv.param("strip_type", param_strip_type_str, std::string("WS2811_STRIP_GRB"));

    // New parameter for SPI
    nh_priv.param("spi_device", param_spi_device, std::string("/dev/spidev4.1"));

    // Legacy params (ignored)
    nh_priv.param("target_frequency", param_freq, 800000);
    nh_priv.param("gpio_pin", param_pin, 21);
    nh_priv.param("dma", param_dma, 10);
    nh_priv.param("invert", param_invert, false);

    if (nh_priv.hasParam("gpio_pin") || nh_priv.hasParam("dma")) {
        ROS_WARN("[ws281x] 'gpio_pin', 'dma', 'invert', 'target_frequency' are ignored on Orange Pi SPI driver. Use 'spi_device'.");
    }

    StripType strip_type = STRIP_GRB;
    auto it = strip_types_map.find(param_strip_type_str);
    if (it != strip_types_map.end()) {
        strip_type = it->second;
    } else {
        ROS_WARN("[ws281x] Unknown strip type: %s, defaulting to GRB", param_strip_type_str.c_str());
    }

    ROS_INFO("[ws281x] Initializing SPI device: %s, LEDs: %d, Type: %s",
             param_spi_device.c_str(), param_led_count, param_strip_type_str.c_str());

    if (!led_driver.init(param_spi_device, param_led_count, strip_type)) {
        ROS_FATAL("[ws281x] Failed to initialize SPI driver. Check permissions (sudo usermod -aG spi $USER) and device path.");
        return 1;
    }

    led_driver.setBrightness((uint8_t)param_brightness);

    // Setup signals
    signal(SIGINT, cleanup);
    signal(SIGTERM, cleanup);

    // Services and Publishers
    auto srv_gamma = nh_priv.advertiseService("set_gamma", setGamma);
    auto srv_leds = nh_priv.advertiseService("set_leds", setLeds);

    led_state_pub = nh_priv.advertise<led_msgs::LEDStateArray>("state", 1, true);

    // Initial publish
    publishLedState();

    ros::spin();

    return 0;
