#include "vox.h"

#define OGT_VOX_IMPLEMENTATION
#include <ogt_vox.h>
#include <corecrt_io.h>

std::vector<u8> load_vox_model(const char* path) {
    /* Load the model file */
    FILE* fp = fopen(path, "rb");
    uint32_t buffer_size = _filelength(_fileno(fp));
    uint8_t* buffer = new uint8_t[buffer_size];
    fread(buffer, buffer_size, 1, fp);
    fclose(fp);

    /* Parse the model file */
    const ogt_vox_scene* scene = ogt_vox_read_scene(buffer, buffer_size);
    delete[] buffer; /* Cleanup */

    /* Grab the first model in the scene */
    const ogt_vox_model* model = scene->models[0];

    std::vector<u8> voxel_data;
    voxel_data.resize((size_t)model->size_x * model->size_y * model->size_z);

    /* Store the voxel data */
    for (uint32_t z = 0; z < model->size_z; z++) {
        for (uint32_t y = 0; y < model->size_y; y++) {
            for (uint32_t x = 0; x < model->size_x; x++) {
                /* In & out voxel index */
                uint32_t vi = x + model->size_x * (y + model->size_z * z);
                uint32_t vo = x + model->size_x * ((model->size_z - 1 - z) + model->size_y * y);

                uint8_t color_index = model->voxel_data[vi];
                bool is_voxel_solid = (color_index != 0);

                if (is_voxel_solid) {
                    voxel_data[vo] = rand() % 255 + 1;
                } else {
                    voxel_data[vo] = 0u;
                }
            }
        }
    }

    /* Free the scene */
    delete scene;
    return voxel_data;
}
