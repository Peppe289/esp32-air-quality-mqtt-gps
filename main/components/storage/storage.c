#include "include/storage.h"
#include "esp_bit_defs.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "time.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/unistd.h>

#define BASE_PATH "/spiffs"

typedef struct storage_callbacks_t {
  void (*storage_state_handler)(uint32_t bit, bool add_bit);
  uint32_t (*system_get_state)(void);
} storage_callbacks_t;

storage_callbacks_t s_storage_event_group = {NULL, NULL};

static const char *TAG = "SPIFFS";

static esp_vfs_spiffs_conf_t s_spiffs_conf = {.base_path = BASE_PATH,
                                              .partition_label = NULL,
                                              .max_files = 5,
                                              .format_if_mount_failed = true};

static const char *s_file_path = BASE_PATH "/offline_data.jsonl";

void storage_register_system_handler(
    void (*storage_state_handler)(uint32_t bit, bool add_bit),
    uint32_t (*system_get_state)(void)) {
  s_storage_event_group.storage_state_handler = storage_state_handler;
  s_storage_event_group.system_get_state = system_get_state;
}

static void set_storage_status(uint32_t bit, bool add_bit) {
  if (s_storage_event_group.storage_state_handler) {
    s_storage_event_group.storage_state_handler(bit, add_bit);
  }
}

/**
 * @brief Initializes the SPIFFS filesystem.
 * * Registers the Virtual File System (VFS) and mounts the partition.
 * If mounting fails and the configuration allows, it will attempt
 * to format the partition.
 * * @note This should be called once during the system initialization phase.
 */
void storage_init_fs() {
  ESP_LOGI(TAG, "Initializing SPIFFS");
  esp_err_t ret = esp_vfs_spiffs_register(&s_spiffs_conf);

  switch (ret) {
  case ESP_OK:
    set_storage_status(STORAGE_SYS_STATUS_INIZIALIZED, true);
    ESP_LOGI(TAG, "Inizialized SPIFFS successful");
    break;
  case ESP_FAIL:
    ESP_LOGE(TAG, "Failed to mount or format filesystem");
    break;
  case ESP_ERR_NOT_FOUND:
    ESP_LOGE(TAG, "Failed to find SPIFFS partition");
    break;
  default:
    ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
  }
}

#include "errno.h"

/**
 * @brief Internal helper to open a file with error handling.
 * * @param type_file_mode The file access mode (e.g., "a", "r", "w").
 * @return FILE* Pointer to the opened file or NULL if the operation failed.
 */
static FILE *storage_open_file(const char *type_file_mode) {
  FILE *s_file;
  s_file = fopen(s_file_path, type_file_mode);
  if (!s_file) {
    ESP_LOGE(TAG, "Failed to open file for %s", type_file_mode);
    ESP_LOGE(TAG, "Failed to open file! Error: %s (Code %d)", strerror(errno), errno);
    return NULL;
  }
  return s_file;
}

/**
 * @brief Flushes and closes an open file pointer.
 * * @param s_file Pointer to the FILE object to be closed.
 */
static void storage_close_file(FILE *s_file) {
  fflush(s_file);
  fclose(s_file);
}

static uint8_t storage_is_mounted() {
  uint8_t ret = esp_spiffs_mounted(s_spiffs_conf.partition_label);
  set_storage_status(STORAGE_SYS_STATUS_FAILED, !ret);
  return ret;
}

/**
 * @brief Appends a string to the offline log file.
 * * Opens the file in append mode, writes the provided string followed
 * by a newline character (\n), and ensures data is flushed to flash
 * before closing.
 * * @param p_str Pointer to the null-terminated string to be persisted.
 * @note This function is stateless (opens and closes the file on every call).
 */
void storage_write_on_file(const char *p_str) {
  FILE *s_file;

  if (!storage_is_mounted()) {
    ESP_LOGE(TAG, "Partition isn't mountend!");
    return;
  }

  if (!(s_file = storage_open_file("a")))
    return;

  if (fprintf(s_file, "%s\n", p_str) < 0)
    ESP_LOGE(TAG, "Failed to write file");
  else
    fflush(s_file);

  storage_close_file(s_file);
}

/**
 * @brief Removes the offline data file from the filesystem.
 * * Checks for the file's existence using stat() before attempting
 * to delete it (unlink).
 */
static void storage_remove_file() {
  // check if file exist.
  if (storage_have_file()) {
    unlink(s_file_path);
  }
}

/**
 * Check if file exist.
 */
uint8_t storage_have_file() {
  struct stat st;
  return !stat(s_file_path, &st);
}

/**
 * @brief Recovers saved data and processes it via a callback function.
 * * Reads the file line by line (JSONL format). For each valid line found,
 * it invokes the provided callback. After the entire file is processed,
 * the file is automatically deleted to free space.
 * * @param mqtt_callback Pointer to a function that accepts a const char* *
 * containing the retrieved JSON record.
 * @warning Ensure the internal line buffer is large enough for the
 * longest expected JSON string.
 */
void storage_recovery_data(uint8_t (*mqtt_callback)(const char *p_json,
                                                    const char *p_topic)) {
  char jsonl[512] = {0};
  FILE *s_file;
  static long last_position = 0;

  ESP_LOGI(TAG, "Starting data recovery from %s", s_file_path);

  if (!storage_is_mounted()) {
    ESP_LOGE(TAG, "Partition isn't mountend!");
    return;
  }

  if (!(s_file = storage_open_file("r")))
    return;

  if (last_position) {
    fseek(s_file, last_position, SEEK_SET);
  }

  while (fgets(jsonl, sizeof(jsonl), s_file) != NULL) {

    if (jsonl[0] == '\n')
      continue;

    jsonl[strcspn(jsonl, "\r\n")] = 0;

    if (jsonl[0] != '\0') {
      // p_topic = NULL is useless. mqtt_callback call a wrapper which use
      // MQTT_RECOVERY_TOPIC as parameter
      if (mqtt_callback(jsonl, NULL)) {
        storage_close_file(s_file);
        return;
      } else {
        last_position = ftell(s_file);
      }
    }

    memset(jsonl, 0, sizeof(jsonl));
  }

  last_position = 0;

  storage_close_file(s_file);
  storage_remove_file();
}