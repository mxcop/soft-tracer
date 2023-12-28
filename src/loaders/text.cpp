#include "text.h"

#include <fstream> /* std::ifstream */
#include <vector>

using namespace std;

string load_text_file(const char* path) {
    /* Open and instantly move to the end of the stream */
    ifstream input_stream(path, ios::ate);

    if (not input_stream.is_open()) {
        return string();
    }

    streamsize size = input_stream.tellg();
    input_stream.seekg(0, ios::beg); /* Move to begin */

    vector<char> bytes(size);
    input_stream.read(bytes.data(), size);
    return string(bytes.data(), size);
}
