/* SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <inttypes.h>
#include <stdio.h>
#include <sys/stat.h>
#include <dirent.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "file_manager.h"
#include "pwm_audio.h"

static const char *TAG = "wav player";

typedef struct {
    // The "RIFF" chunk descriptor
    uint8_t ChunkID[4];
    int32_t ChunkSize;
    uint8_t Format[4];
    // The "fmt" sub-chunk
    uint8_t Subchunk1ID[4];
    int32_t Subchunk1Size;
    int16_t AudioFormat;
    int16_t NumChannels;
    int32_t SampleRate;
    int32_t ByteRate;
    int16_t BlockAlign;
    int16_t BitsPerSample;
    // The "data" sub-chunk
    uint8_t Subchunk2ID[4];
    int32_t Subchunk2Size;
} wav_header_t;

#ifdef CONFIG_STORAGE_SDCARD
static char **g_file_list = NULL;
static uint16_t g_file_num = 0;
#endif

// Button pin for ESP32-C3 (boot button)
#define GPIO_BUTTON 9

// Speed multipliers (15%, 25%, 50%, 80%, 100%, 120%, 150%) - sorted ascending
#define NUM_SPEEDS 7
static const float speed_multipliers[NUM_SPEEDS] = {0.15f, 0.25f, 0.5f, 0.8f, 1.0f, 1.2f, 1.5f};
static const char* speed_labels[NUM_SPEEDS] = {"15%", "25%", "50%", "80%", "100%", "120%", "150%"};

// Shared variables for speed control
static volatile int current_speed_index = 4; // Start at 100% (index 4 in the new array)
static SemaphoreHandle_t speed_mutex = NULL;

#ifdef CONFIG_IDF_TARGET_ESP32
#define GPIO_AUDIO_OUTPUT_L 25
#define GPIO_AUDIO_OUTPUT_R 26
#elif defined CONFIG_IDF_TARGET_ESP32S2
#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2
#elif defined CONFIG_IDF_TARGET_ESP32S3
#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2
#elif defined CONFIG_IDF_TARGET_ESP32C3
#define GPIO_AUDIO_OUTPUT_L 3
#define GPIO_AUDIO_OUTPUT_R 2
#else
#define GPIO_AUDIO_OUTPUT_L 1
#define GPIO_AUDIO_OUTPUT_R 2
#endif

static int get_current_speed_index(void)
{
    int index;
    if (speed_mutex != NULL) {
        xSemaphoreTake(speed_mutex, portMAX_DELAY);
        index = current_speed_index;
        xSemaphoreGive(speed_mutex);
    } else {
        index = current_speed_index;
    }
    return index;
}

static esp_err_t play_wav(const char *filepath)
{
    FILE *fd = NULL;
    struct stat file_stat;

    if (stat(filepath, &file_stat) == -1) {
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "file stat info: %s (%ld bytes)...", filepath, file_stat.st_size);
    fd = fopen(filepath, "r");

    if (NULL == fd) {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        return ESP_FAIL;
    }
    const size_t chunk_size = 4096;
    uint8_t *buffer = malloc(chunk_size);

    if (NULL == buffer) {
        ESP_LOGE(TAG, "audio data buffer malloc failed");
        fclose(fd);
        return ESP_FAIL;
    }

    /**
     * read head of WAV file
     */
    wav_header_t wav_head;
    int len = fread(&wav_head, 1, sizeof(wav_header_t), fd);
    if (len <= 0) {
        ESP_LOGE(TAG, "Read wav header failed");
        fclose(fd);
        free(buffer);
        return ESP_FAIL;
    }
    if (NULL == strstr((char *)wav_head.Subchunk1ID, "fmt") &&
            NULL == strstr((char *)wav_head.Subchunk2ID, "data")) {
        ESP_LOGE(TAG, "Header of wav format error");
        fclose(fd);
        free(buffer);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "frame_rate= %"PRIi32", ch=%d, width=%d", wav_head.SampleRate, wav_head.NumChannels, wav_head.BitsPerSample);

    // Store the position where audio data starts (right after header)
    long data_start_pos = ftell(fd);

    // Store original sample rate
    int32_t original_sample_rate = wav_head.SampleRate;
    int last_speed_index = get_current_speed_index();
    int32_t current_sample_rate = (int32_t)(original_sample_rate * speed_multipliers[last_speed_index]);

    pwm_audio_set_param(current_sample_rate, wav_head.BitsPerSample, wav_head.NumChannels);
    pwm_audio_start();

    ESP_LOGI(TAG, "Starting continuous loop playback at %s speed (sample rate: %"PRIi32")...",
             speed_labels[last_speed_index], current_sample_rate);

    /**
     * read wave data of WAV file in continuous loop
     */
    size_t cnt;

    while (1) {
        // Check if speed has changed
        int speed_index = get_current_speed_index();
        if (speed_index != last_speed_index) {
            ESP_LOGI(TAG, "Speed change requested: %s -> %s",
                     speed_labels[last_speed_index], speed_labels[speed_index]);

            // Stop audio before changing parameters
            pwm_audio_stop();
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to ensure stop is complete

            // Calculate and set new sample rate
            current_sample_rate = (int32_t)(original_sample_rate * speed_multipliers[speed_index]);
            esp_err_t param_ret = pwm_audio_set_param(current_sample_rate, wav_head.BitsPerSample, wav_head.NumChannels);
            if (param_ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to set audio params: %s", esp_err_to_name(param_ret));
            }

            // Restart audio
            pwm_audio_start();
            vTaskDelay(pdMS_TO_TICKS(10)); // Small delay to ensure start is complete

            ESP_LOGI(TAG, "Speed changed to %s (sample rate: %"PRIi32")",
                     speed_labels[speed_index], current_sample_rate);
            last_speed_index = speed_index;
        }

        /* Read file in chunks into the scratch buffer */
        len = fread(buffer, 1, chunk_size, fd);
        if (len <= 0) {
            // Reached end of file, seek back to start of audio data to loop
            fseek(fd, data_start_pos, SEEK_SET);
            continue;
        }
        pwm_audio_write(buffer, len, &cnt, 1000 / portTICK_PERIOD_MS);
    }

    // This code should never be reached, but included for completeness
    pwm_audio_stop();
    fclose(fd);
    free(buffer);
    return ESP_OK;
}

static void button_task(void *arg)
{
    bool last_level = 1; // Button is normally high (pulled up)
    TickType_t last_press_time = 0;
    const TickType_t debounce_ticks = pdMS_TO_TICKS(100);
    TickType_t last_log_time = 0;
    const TickType_t log_interval = pdMS_TO_TICKS(2000); // Log every 2 seconds for debugging

    // Log initial button state
    bool initial_level = gpio_get_level(GPIO_BUTTON);
    ESP_LOGI(TAG, "Button task started. Initial GPIO %d level: %d (1=high/released, 0=low/pressed)",
             GPIO_BUTTON, initial_level);

    while (1) {
        // Poll button state (more reliable than interrupts)
        bool current_level = gpio_get_level(GPIO_BUTTON);
        TickType_t now = xTaskGetTickCount();

        // Periodic logging for debugging (every 2 seconds)
        if ((now - last_log_time) > log_interval) {
            ESP_LOGI(TAG, "Button GPIO %d state: %d (polling active)", GPIO_BUTTON, current_level);
            last_log_time = now;
        }

        // Check for button press (falling edge) with debouncing
        if (current_level == 0 && last_level == 1) {
            // Button pressed (went from high to low)
            ESP_LOGI(TAG, "Button state change detected: HIGH -> LOW");
            if ((now - last_press_time) > debounce_ticks) {
                // Button pressed - cycle to next speed
                if (speed_mutex != NULL) {
                    xSemaphoreTake(speed_mutex, portMAX_DELAY);
                    current_speed_index = (current_speed_index + 1) % NUM_SPEEDS;
                    ESP_LOGI(TAG, "*** BUTTON PRESSED! Speed changed to %s ***", speed_labels[current_speed_index]);
                    xSemaphoreGive(speed_mutex);
                } else {
                    ESP_LOGE(TAG, "Speed mutex is NULL!");
                }
                last_press_time = now;
            } else {
                ESP_LOGD(TAG, "Button press ignored (debounce)");
            }
        }

        // Also detect button release for debugging
        if (current_level == 1 && last_level == 0) {
            ESP_LOGD(TAG, "Button released (LOW -> HIGH)");
        }

        last_level = current_level;
        vTaskDelay(pdMS_TO_TICKS(20)); // Poll every 20ms
    }
}

// Note: Using polling instead of ISR for more reliable button detection

void app_main()
{
    esp_err_t ret;

    ESP_LOGI(TAG, "[APP] Startup..");
    ESP_LOGI(TAG, "[APP] Free memory: %"PRIu32" bytes", esp_get_free_heap_size());
    ESP_LOGI(TAG, "[APP] IDF version: %s", esp_get_idf_version());

    // Create mutex for speed control
    speed_mutex = xSemaphoreCreateMutex();
    if (speed_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create speed mutex");
        return;
    }

    // Configure button GPIO
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,  // Using polling, not interrupts
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << GPIO_BUTTON),
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    esp_err_t gpio_ret = gpio_config(&io_conf);
    if (gpio_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure button GPIO: %s", esp_err_to_name(gpio_ret));
    } else {
        ESP_LOGI(TAG, "Button GPIO %d configured successfully", GPIO_BUTTON);
    }

    // Read and log initial button state
    bool button_state = gpio_get_level(GPIO_BUTTON);
    ESP_LOGI(TAG, "Initial button state on GPIO %d: %s (1=high/released, 0=low/pressed)",
             GPIO_BUTTON, button_state ? "HIGH" : "LOW");

    // Create button monitoring task
    TaskHandle_t button_task_handle = NULL;
    BaseType_t task_ret = xTaskCreate(button_task, "button_task", 2048, NULL, 5, &button_task_handle);
    if (task_ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create button task");
    } else {
        ESP_LOGI(TAG, "Button task created. Press button on GPIO %d to cycle speeds: 15%% -> 25%% -> 50%% -> 80%% -> 100%% -> 120%% -> 150%%", GPIO_BUTTON);
    }

    pwm_audio_config_t pac = {
        .duty_resolution    = LEDC_TIMER_10_BIT,
        .gpio_num_left      = GPIO_AUDIO_OUTPUT_L,
        .ledc_channel_left  = LEDC_CHANNEL_0,
        .gpio_num_right     = GPIO_AUDIO_OUTPUT_R,
        .ledc_channel_right = LEDC_CHANNEL_1,
        .ledc_timer_sel     = LEDC_TIMER_0,
        .ringbuf_len        = 1024 * 8,
#if ESP_IDF_VERSION < ESP_IDF_VERSION_VAL(5, 0, 0)
        .tg_num             = TIMER_GROUP_0,
        .timer_num          = TIMER_0,
#endif
    };
    pwm_audio_init(&pac);

#ifdef CONFIG_STORAGE_SDCARD
    ret = fm_sdcard_init();
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "sdcard initial failed, exit");
        return;
    }
    fm_print_dir(MOUNT_POINT, 2);
    fm_file_table_create(&g_file_list, &g_file_num, ".wav");
    for (size_t i = 0; i < g_file_num; i++) {
        ESP_LOGI(TAG, "have file [%d:%s]", i, g_file_list[i]);
    }
    if (0 == g_file_num) {
        ESP_LOGW(TAG, "Can't found any wav file in sdcard!");
        return;
    }

    // Play the first file in continuous loop
    if (g_file_num > 0) {
        ESP_LOGI(TAG, "Start to play [0:%s] in continuous loop", g_file_list[0]);
        char path_buf[256] = {0};
        sprintf(path_buf, "%s/%s", MOUNT_POINT, g_file_list[0]);
        play_wav(path_buf);
    }
    fm_file_table_free(&g_file_list, g_file_num);

#else
    ret = fm_spiffs_init();
    if (ESP_OK != ret) {
        ESP_LOGE(TAG, "sdcard initial failed, exit");
        return;
    }
    play_wav(MOUNT_POINT "/sample.wav");
#endif

}
