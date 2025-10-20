#pragma once
#include <string>
#include <vector>
#include <stdint.h>
#include <stdio.h>
#include <dirent.h>
#include <cstring>

class WavFileWriter {
public:
    static std::string GetNextFilename(const std::string& dir = "/spiffs/") {
        int max_index = 0;
        DIR* d = opendir(dir.c_str());
        if (d) {
            struct dirent* entry;
            while ((entry = readdir(d)) != nullptr) {
                std::string name(entry->d_name);
                if (name.find("record_") == 0 && name.find(".wav") != std::string::npos) {
                    int idx = 0;
                    sscanf(name.c_str(), "record_%d.wav", &idx);
                    if (idx > max_index) max_index = idx;
                }
            }
            closedir(d);
        }
        char buf[64];
        snprintf(buf, sizeof(buf), "%srecord_%03d.wav", dir.c_str(), max_index + 1);
        return std::string(buf);
    }

    static bool WriteWav(const std::string& filename, const std::vector<int16_t>& pcm, int sample_rate = 16000, int channels = 1) {
        FILE* fp = fopen(filename.c_str(), "wb");
        if (!fp) return false;
        int data_size = pcm.size() * sizeof(int16_t);
        int header_size = 44;
        // WAV header
        uint8_t header[44] = {0};
        memcpy(header, "RIFF", 4);
        *(uint32_t*)(header + 4) = data_size + 36;
        memcpy(header + 8, "WAVEfmt ", 8);
        *(uint32_t*)(header + 16) = 16;
        *(uint16_t*)(header + 20) = 1; // PCM
        *(uint16_t*)(header + 22) = channels;
        *(uint32_t*)(header + 24) = sample_rate;
        *(uint32_t*)(header + 28) = sample_rate * channels * 2;
        *(uint16_t*)(header + 32) = channels * 2;
        *(uint16_t*)(header + 34) = 16;
        memcpy(header + 36, "data", 4);
        *(uint32_t*)(header + 40) = data_size;
        fwrite(header, 1, header_size, fp);
        fwrite(pcm.data(), 1, data_size, fp);
        fclose(fp);
        return true;
    }
};
