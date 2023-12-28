#pragma once

#include <string>

/**
 * @brief Loads a text file relative to the working directory.
 * 
 * @param path Path relative to the working directory.
 * @return An emtpy string in case of failure.
 */
std::string load_text_file(const char* path);
